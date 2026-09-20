import http.client
import json
import tempfile
import threading
import unittest

from openpilot.selfdrive.diagnostics.manager import ReportStore
from openpilot.selfdrive.diagnostics.mcp import MCPServer
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence


class FakeManager:
  def __init__(self, root):
    self.store = ReportStore(root)
    self.calls = 0
    self.block = False
    self.started = threading.Event()
    self.cancelled = threading.Event()

  def scan(self, args, cancel):
    self.calls += 1
    self.started.set()
    if self.block:
      if cancel.wait(5):
        self.cancelled.set()
    return self.store.save(f'{self.calls:032x}', evidence())


class TestMCP(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.manager = FakeManager(self.tmp.name)
    self.server = MCPServer(('127.0.0.1', 0), manager=self.manager)
    self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
    self.thread.start()
    self.sid = None

  def tearDown(self):
    self.server.cancel_all()
    self.server.shutdown()
    self.server.server_close()
    self.thread.join(timeout=2)
    for job in self.server.jobs.values():
      job.done.wait(2)
    self.tmp.cleanup()

  def request(self, message=None, method='POST', headers=None, read=True):
    connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=5)
    base = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}
    if self.sid:
      base.update({'MCP-Session-Id': self.sid, 'MCP-Protocol-Version': '2025-11-25'})
    base.update(headers or {})
    connection.request(method, '/mcp', json.dumps(message) if message is not None else None, base)
    response = connection.getresponse()
    if not read:
      return connection, response
    status, response_headers, data = response.status, dict(response.getheaders()), response.read().decode()
    connection.close()
    return status, response_headers, data

  def initialize(self, version='2025-11-25'):
    status, headers, data = self.request({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize',
                                         'params': {'protocolVersion': version, 'capabilities': {}, 'clientInfo': {'name': 'test', 'version': '1'}}})
    self.assertEqual(status, 200)
    self.sid = headers['MCP-Session-Id']
    self.assertEqual(json.loads(data)['result']['protocolVersion'], '2025-11-25')
    self.assertEqual(self.request({'jsonrpc': '2.0', 'method': 'notifications/initialized'})[0], 202)

  def rpc(self, method, params=None, req_id=2):
    return {'jsonrpc': '2.0', 'id': req_id, 'method': method, 'params': params or {}}

  def test_lifecycle_tools_and_version_negotiation(self):
    self.initialize('2099-01-01')
    status, _, data = self.request(self.rpc('tools/list'))
    self.assertEqual(status, 200)
    self.assertEqual([t['name'] for t in json.loads(data)['result']['tools']], ['scan_vehicle', 'get_scan_evidence'])
    self.assertEqual(self.request(self.rpc('ping'))[0], 200)
    self.assertEqual(self.request(method='GET')[0], 405)
    self.assertEqual(self.manager.calls, 0)

  def test_scan_stream_and_evidence_without_rescan(self):
    self.initialize()
    status, headers, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715'}}))
    self.assertEqual(status, 200)
    self.assertEqual(headers['Content-Type'], 'text/event-stream')
    messages = [json.loads(line[6:]) for line in data.splitlines() if line.startswith('data: ')]
    scan_id = messages[-1]['result']['structuredContent']['scan_id']
    _, _, body = self.request(self.rpc('tools/call', {'name': 'get_scan_evidence', 'arguments': {'scan_id': scan_id}}, 3))
    self.assertEqual(json.loads(body)['result']['structuredContent']['report']['scan_id'], scan_id)
    self.assertEqual(self.manager.calls, 1)
    self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 2))
    self.assertEqual(self.manager.calls, 1)  # same session/request ID is replayed

  def test_cancel_and_busy(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 10), read=False)
    self.assertTrue(self.manager.started.wait(1))
    _, _, busy = self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 11))
    self.assertTrue(json.loads(busy)['result']['isError'])
    self.assertEqual(self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 10}})[0], 202)
    self.assertTrue(self.manager.cancelled.wait(1))
    response.read()
    connection.close()

  def test_cancellation_does_not_cross_sessions(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 10), read=False)
    original = self.sid
    self.sid = None
    self.initialize()
    self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 10}})
    self.assertFalse(self.server.jobs[(original, 10)].cancel.is_set())
    self.sid = original
    self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 10}})
    response.read()
    connection.close()

  def test_security_headers_and_no_unsafe_tools(self):
    self.assertEqual(self.request(self.rpc('ping'), headers={'Host': 'evil.example'})[0], 403)
    self.assertEqual(self.request(self.rpc('ping'), headers={'Origin': 'https://evil.example'})[0], 403)
    self.server.token = 'demo-secret'
    self.assertEqual(self.request(self.rpc('ping'))[0], 401)
    self.server.token = None
    self.initialize()
    self.assertEqual(self.request(self.rpc('ping'), headers={'MCP-Protocol-Version': 'invalid'})[0], 400)
    _, _, data = self.request(self.rpc('tools/call', {'name': 'clear_codes'}))
    self.assertIn('error', json.loads(data))
    _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'shell': 'anything'}}))
    self.assertIn('error', json.loads(data))
    self.assertEqual(self.manager.calls, 0)

  def test_invalid_jsonrpc_types_do_not_crash_handler(self):
    self.initialize()
    for bad in ([], {'jsonrpc': '2.0', 'id': [], 'method': 'ping'}, {'jsonrpc': '2.0', 'id': True, 'method': 'ping'}):
      self.assertEqual(self.request(bad)[0], 400)
    for args in ({'broad': 'yes'}, {'target': '0x7df'}, {'details': 1}):
      _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': args}))
      self.assertTrue(json.loads(data)['result']['isError'])
    self.assertEqual(self.manager.calls, 0)

  def test_delete_session_cancels_owned_work(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 10), read=False)
    self.assertEqual(self.request(method='DELETE')[0], 200)
    self.assertTrue(self.manager.cancelled.wait(1))
    response.read()
    connection.close()
    self.assertEqual(self.request(self.rpc('ping'))[0], 404)

  def test_non_loopback_bind_refused(self):
    with self.assertRaises(ValueError):
      MCPServer(('0.0.0.0', 0), manager=self.manager)
