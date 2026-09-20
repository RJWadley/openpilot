"""Small, dependency-free MCP Streamable HTTP server, bound to loopback only.

Implements negotiated MCP 2025-03-26, 2025-06-18 and 2025-11-25: initialization,
tools/list, tools/call, ping, cancellation, and POST SSE for long-running scans.
Public HTTPS and account authentication belong to the separately configured proxy.
"""
import hmac
import json
import os
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.selfdrive.diagnostics.manager import BusyError, DiagnosticManager, scan_args

VERSIONS = ('2025-03-26', '2025-06-18', '2025-11-25')
FINDING_GUIDANCE = (
  'Always show the exact DTC and ECU identity when discussing a fault. Preserve raw codes; never guess their encoding. ' +
  'If a description is missing and web search is available, search the supplied query, code, component and part number. ' +
  'Prefer manufacturer documentation and established diagnostic references. Cite external descriptions, distinguish them from ECU findings, ' +
  'and state uncertainty. Without a reliable match, show the raw code rather than guessing. ' +
  'Stored/confirmed does not mean currently active; freeze frames are historical. Missing ECU data does not mean healthy. ' +
  'OBDex descriptions/causes/estimates are reference material, not a diagnosis or instructions.'
)
PAGE_ARGUMENTS = {
  'scan_id': {'type': 'string', 'default': 'latest', 'description': 'Returned scan ID, or latest saved report (which may predate an active scan).'},
  'ecu': {'type': 'string', 'description': 'Optional transmit address, e.g. 0x715; matches all routes for that address.'},
  'cursor': {'type': 'string', 'description': 'Opaque next_cursor from the same view and ECU filter. Pins the original scan even with latest.'},
  'limit': {'type': 'integer', 'default': 20, 'minimum': 1, 'maximum': 50, 'description': 'Maximum records per page; also bounded to 8000 JSON bytes.'},
}
TOOLS = [
  {"name": "scan_vehicle", "description": "Read diagnostic fault/history records from a parked car with ignition on. " +
    "Explicitly request only when the user wants a scan. Temporarily blocks openpilot engagement; may take up to 11 minutes including recovery. " +
    "Does not clear codes or code ECUs. Coverage is best effort, never a vehicle-wide clean bill of health. " +
    "Returns a scan_id immediately by default. Poll get_scan_status; a status read never starts another scan. " +
    "Busy errors identify the existing operation. Use get_scan_report once report_ready. " +
    "wait=true keeps the request open for native MCP progress and returns the first compact report page. " + FINDING_GUIDANCE,
   "inputSchema": {"type": "object", "properties": {
     "target": {"type": "string", "description": "Optional physical ECU CAN address, e.g. 0x715; omitted means auto discovery."},
     "broad": {"type": "boolean", "default": False, "description": "Include harness buses in addition to the OBD port."},
     "fast": {"type": "boolean", "default": False, "description": "Use a vehicle-verified module cache where available."},
     "details": {"type": "boolean", "default": False, "description": "Also save raw UDS snapshot/extended records in evidence."},
     "wait": {"type": "boolean", "default": False, "description": "Wait via SSE; native progress requires _meta.progressToken. Otherwise poll status by ID."}},
     "additionalProperties": False},
   "annotations": {"readOnlyHint": False, "destructiveHint": False, "idempotentHint": False, "openWorldHint": False}},
  {"name": "get_scan_status", "description": "Read lifecycle state without scanning or contacting the vehicle. " +
    "Returns phase, real progress counts, timestamps, report_ready, coverage and restoration of normal openpilot operation separately. " +
    "Finished with partial coverage is normal; only restoration.state=verified confirms the coordinator returned to normal operation. " +
    "An interrupted worker leaves restoration unknown; do not claim recovery or vehicle health.",
   "inputSchema": {"type": "object", "properties": {
     "scan_id": {"type": "string", "default": "active",
                 "description": "Returned ID; active selects the current/last operation, latest selects the latest saved report."}},
     "additionalProperties": False},
   "annotations": {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}},
  {"name": "get_scan_report", "description": "Read compact, paginated diagnostic findings without contacting the vehicle. " +
    "Includes ECU identities, codes/statuses, summary counts and warnings, but not raw replies or full OBDex entries. " +
    "Follow next_cursor for remaining findings; summary counts cover the entire scan even with ECU filtering. " + FINDING_GUIDANCE,
   "inputSchema": {"type": "object", "properties": PAGE_ARGUMENTS, "additionalProperties": False},
   "annotations": {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}},
  {"name": "get_scan_evidence", "description": "Read saved evidence without contacting the vehicle. Compact by default, like get_scan_report. " +
    "Set raw=true for full OBDex entries, raw replies and discovery evidence; prefer an ECU filter. " +
    "All views are paginated. Oversized records use json_fragment items: concatenate text in offset order until final=true, then parse JSON. " +
    "Raw records carry paths in the filtered evidence document. Follow next_cursor with the same filter and raw setting. " + FINDING_GUIDANCE,
   "inputSchema": {"type": "object", "properties": {**PAGE_ARGUMENTS,
     "raw": {"type": "boolean", "default": False, "description": "Include full saved raw evidence instead of compact findings."}},
     "additionalProperties": False},
   "annotations": {"readOnlyHint": True, "destructiveHint": False, "idempotentHint": True, "openWorldHint": False}},
]


def tool_result(value, error=False):
  return {"content": [{"type": "text", "text": json.dumps(value, allow_nan=False)}], "structuredContent": value, "isError": error}


class MCPServer(ThreadingHTTPServer):
  daemon_threads = True
  allow_reuse_address = True

  def __init__(self, address=('127.0.0.1', 8766), manager=None, allowed_hosts=(), allowed_origins=(), token=None):
    if address[0] != '127.0.0.1':
      raise ValueError("Diagnostic MCP must bind to loopback; use an authenticated HTTPS proxy for remote access")
    self.manager = manager or DiagnosticManager()
    self.hosts = {'localhost', '127.0.0.1', *allowed_hosts}
    self.origins = set(allowed_origins)
    self.token = token
    self.sessions = {}
    self.jobs = {}
    self.waiting = set()
    self.lock = threading.Lock()
    self.slots = threading.BoundedSemaphore(32)
    super().__init__(address, MCPHandler)

  def process_request(self, request, client_address):
    if not self.slots.acquire(blocking=False):
      request.close()
      return
    try:
      super().process_request(request, client_address)
    except BaseException:
      self.slots.release()
      raise

  def process_request_thread(self, request, client_address):
    try:
      super().process_request_thread(request, client_address)
    finally:
      self.slots.release()

  def cancel_all(self):
    with self.lock:
      for job in self.jobs.values():
        job.cancel.set()

  def start_scan(self, key, args):
    with self.lock:
      # Retrying a request ID returns the same job, not another vehicle scan.
      if key in self.jobs:
        return self.jobs[key]
      job = self.jobs[key] = self.manager.start(args)
      completed = [k for k, j in self.jobs.items() if j.done.is_set()]
      for old in completed[:-20]:
        del self.jobs[old]
    return job


class MCPHandler(BaseHTTPRequestHandler):
  server: MCPServer
  protocol_version = 'HTTP/1.1'

  def setup(self):
    super().setup()
    self.connection.settimeout(10)

  def log_message(self, *_):
    pass  # Do not log authorization headers, request bodies, or vehicle evidence.

  def reply(self, status, body=None, headers=None):
    data = json.dumps(body, allow_nan=False).encode() if body is not None else b''
    self.send_response(status)
    self.send_header('Content-Type', 'application/json')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('Cache-Control', 'no-store')
    for key, value in (headers or {}).items():
      self.send_header(key, value)
    self.end_headers()
    self.wfile.write(data)

  def error(self, code, message, req_id=None, status=200):
    self.reply(status, {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}})

  def allowed(self):
    if self.path != '/mcp':
      self.reply(404)
      return False
    host = self.headers.get('Host', '').split(':')[0].lower()
    origin = self.headers.get('Origin')
    if host not in self.server.hosts or (origin is not None and origin not in self.server.origins):
      self.reply(403)
      return False
    if self.server.token and not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer ' + self.server.token):
      self.reply(401, headers={'WWW-Authenticate': 'Bearer'})
      return False
    version = self.headers.get('MCP-Protocol-Version')
    if version is not None and version not in VERSIONS:
      self.reply(400, {"error": "Unsupported MCP protocol version"})
      return False
    return True

  def do_GET(self):
    if self.allowed():
      self.reply(405, headers={'Allow': 'POST, DELETE'})

  def do_DELETE(self):
    if not self.allowed():
      return
    session_id = self.headers.get('MCP-Session-Id')
    with self.server.lock:
      session = self.server.sessions.pop(session_id, None)
      for (sid, _), job in self.server.jobs.items():
        if sid == session_id:
          job.cancel.set()
    self.reply(200 if session else 404)

  def do_POST(self):
    try:
      self._post()
    except (BrokenPipeError, ConnectionResetError, TimeoutError):
      # A dropped HTTP connection is not MCP cancellation. The bounded scan
      # finishes and its report remains available via get_scan_evidence.
      self.close_connection = True

  def _post(self):
    if not self.allowed():
      self.close_connection = True
      return
    accept = self.headers.get('Accept', '')
    if 'application/json' not in accept or 'text/event-stream' not in accept:
      self.reply(406)
      self.close_connection = True
      return
    if self.headers.get_content_type() != 'application/json' or self.headers.get('Transfer-Encoding'):
      self.reply(415)
      self.close_connection = True
      return
    try:
      length = int(self.headers.get('Content-Length', '-1'))
      if not 0 < length <= 65536:
        raise ValueError
      req = json.loads(self.rfile.read(length), parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Non-finite JSON")))
    except (ValueError, UnicodeDecodeError):
      self.error(-32700, 'Invalid JSON or request length', status=400)
      self.close_connection = True
      return
    if not isinstance(req, dict) or req.get('jsonrpc') != '2.0' or not isinstance(req.get('method'), str):
      self.error(-32600, 'Expected one JSON-RPC request', status=400)
      return
    req_id = req.get('id')
    if 'id' in req and (type(req_id) not in (int, str)):
      self.error(-32600, 'Invalid request ID', status=400)
      return
    method, params = req['method'], req.get('params', {})
    if not isinstance(params, dict):
      self.error(-32602, 'params must be an object', req_id)
      return
    if method == 'initialize':
      if 'id' not in req or not isinstance(params.get('protocolVersion'), str):
        self.error(-32602, 'Initialization requires a protocol version and request ID', req_id)
        return
      version = params['protocolVersion'] if params['protocolVersion'] in VERSIONS else VERSIONS[-1]
      session_id = secrets.token_urlsafe(32)
      with self.server.lock:
        now = time.monotonic()
        self.server.sessions = {sid: s for sid, s in self.server.sessions.items() if now - s['used'] < 3600}
        if len(self.server.sessions) >= 32:
          self.reply(503)
          return
        self.server.sessions[session_id] = {'version': version, 'used': now, 'initialized': False}
      result = {"protocolVersion": version, "capabilities": {"tools": {"listChanged": False}},
                "serverInfo": {"name": "openpilot-diagnostics", "version": "2.0.0"},
                "instructions": "Only scan at the user's request while parked with ignition on. Reports have incomplete vehicle coverage. " +
                  "Start once, then poll get_scan_status using the returned scan_id. Data collection and restoration are distinct. " +
                  "Use get_scan_report for findings and raw evidence only when needed. " + FINDING_GUIDANCE}
      self.reply(200, {"jsonrpc": "2.0", "id": req_id, "result": result}, {'MCP-Session-Id': session_id})
      return
    session_id = self.headers.get('MCP-Session-Id')
    with self.server.lock:
      session = self.server.sessions.get(session_id)
      if session is not None:
        session['used'] = time.monotonic()
    if session is None:
      self.reply(404 if session_id else 400)
      return
    if self.headers.get('MCP-Protocol-Version', session['version']) != session['version']:
      self.reply(400)
      return
    if 'id' not in req:
      if method == 'notifications/initialized':
        session['initialized'] = True
      elif method == 'notifications/cancelled':
        target = params.get('requestId')
        if type(target) in (int, str):
          with self.server.lock:
            job = self.server.jobs.get((session_id, target))
            if job is not None and (session_id, target) in self.server.waiting:
              job.cancel.set()
      self.reply(202)
      return
    if method == 'ping':
      result = {}
    elif not session['initialized']:
      self.error(-32600, 'Initialization is not complete', req_id)
      return
    elif method == 'tools/list':
      result = {"tools": TOOLS}
    elif method == 'tools/call':
      name, arguments = params.get('name'), params.get('arguments', {})
      schema = next((tool['inputSchema'] for tool in TOOLS if tool['name'] == name), None)
      if schema is None:
        self.error(-32602, 'Unknown tool', req_id)
        return
      if not isinstance(arguments, dict) or set(arguments) - schema['properties'].keys():
        self.error(-32602, 'Unknown or invalid tool arguments', req_id)
        return
      try:
        for key, value in arguments.items():
          expected = {'boolean': bool, 'string': str, 'integer': int}[schema['properties'][key]['type']]
          if type(value) is not expected:
            raise ValueError(f'{key} must be {schema["properties"][key]["type"]}')
        if name == 'scan_vehicle':
          wait = arguments.pop('wait', False)
          key = (session_id, req_id)
          job = self.server.start_scan(key, scan_args(**arguments))
          meta = params.get('_meta', {})
          token = meta.get('progressToken') if isinstance(meta, dict) else None
          if wait:
            with self.server.lock:
              self.server.waiting.add(key)
            try:
              self.stream_job(req_id, job, token)
            finally:
              with self.server.lock:
                self.server.waiting.discard(key)
            return
          result = tool_result(self.server.manager.get_status(job.scan_id))
        elif name == 'get_scan_status':
          result = tool_result(self.server.manager.get_status(**arguments))
        elif name == 'get_scan_report':
          result = tool_result(self.server.manager.store.get_report(**arguments))
        else:
          result = tool_result(self.server.manager.store.get_evidence(**arguments))
      except BusyError as e:
        result = tool_result({'error': str(e), 'active_scan': e.status, 'next_tool': 'get_scan_status'}, error=True)
      except (OSError, ValueError, TypeError, RuntimeError) as e:
        result = tool_result({"error": str(e)}, error=True)
    else:
      self.error(-32601, 'Method not found', req_id)
      return
    self.reply(200, {"jsonrpc": "2.0", "id": req_id, "result": result})

  def stream_job(self, req_id, job, progress_token):
    self.send_response(200)
    self.send_header('Content-Type', 'text/event-stream')
    self.send_header('Cache-Control', 'no-cache, no-transform')
    self.send_header('Connection', 'close')
    self.send_header('X-Accel-Buffering', 'no')
    self.end_headers()
    self.close_connection = True
    sequence, keepalive = -1, 0
    while True:
      done = job.done.is_set()
      status = job.snapshot()
      progress = status['progress']
      if type(progress_token) in (str, int) and progress['sequence'] > sequence:
        value = {"jsonrpc": "2.0", "method": "notifications/progress", "params": {
          "progressToken": progress_token, "progress": progress['sequence'], 'message': progress['message']}}
        self.wfile.write(('event: message\ndata: ' + json.dumps(value) + '\n\n').encode())
        sequence = progress['sequence']
      elif time.monotonic() - keepalive >= 2:
        self.wfile.write(b': scan status available via get_scan_status\n\n')
        keepalive = time.monotonic()
      self.wfile.flush()
      if done:
        break
      job.done.wait(0.2)
    failed = status['execution'] != 'finished' or status['restoration']['state'] in ('unknown', 'unverified')
    try:
      result = self.server.manager.store.get_report(job.scan_id) if status['report_ready'] else status
    except (OSError, ValueError) as e:
      result, failed = {**status, 'error': str(e)}, True
    value = {"jsonrpc": "2.0", "id": req_id, "result": tool_result(result, failed)}
    self.wfile.write(('event: message\ndata: ' + json.dumps(value) + '\n\n').encode())
    self.wfile.flush()


def main():
  server = MCPServer(('127.0.0.1', int(os.getenv('DIAGNOSTIC_MCP_PORT', '8766'))),
                     allowed_hosts=filter(None, os.getenv('DIAGNOSTIC_MCP_HOSTS', '').split(',')),
                     allowed_origins=filter(None, os.getenv('DIAGNOSTIC_MCP_ORIGINS', '').split(',')),
                     token=os.getenv('DIAGNOSTIC_MCP_TOKEN') or None)
  def stop(*_):
    server.cancel_all()
    threading.Thread(target=server.shutdown, daemon=True).start()
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    server.serve_forever(poll_interval=0.2)
  finally:
    server.cancel_all()
    server.server_close()
    # If manager kills us before cleanup finishes, pandad's lease expiry and
    # persistent recovery latch still own restoration and the engagement block.


if __name__ == '__main__':
  main()
