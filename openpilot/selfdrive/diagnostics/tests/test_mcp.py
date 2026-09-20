import http.client
import json
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from openpilot.selfdrive.diagnostics.manager import DiagnosticManager, ReportStore
from openpilot.selfdrive.diagnostics.mcp import MCPServer
from openpilot.selfdrive.diagnostics.tests.test_lifecycle import ParkedPanda
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence


class FakeManager(DiagnosticManager):
  def __init__(self, root):
    self.calls = 0
    self.block = False
    self.started = threading.Event()
    self.cancelled = threading.Event()
    self.release = threading.Event()
    super().__init__(ReportStore(root), self.transport)

  def transport(self, cancel):
    self.calls += 1
    owner = self
    class Adapter(ParkedPanda):
      def health(self):
        owner.started.set()
        if owner.block and cancel.wait(5):
          owner.cancelled.set()
        owner.release.set()
        return super().health()
    return Adapter(cancel, self.started, self.release)


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

  def request(self, message=None, method='POST', headers=None, read=True, timeout=5):
    connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=timeout)
    base = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}
    if self.sid:
      base.update({'MCP-Session-Id': self.sid, 'MCP-Protocol-Version': '2025-11-25'})
    base.update(headers or {})
    connection.request(method, '/mcp', json.dumps(message) if message is not None else None, base)
    response = connection.getresponse()
    if not read:
      return connection, response
    try:
      status, response_headers, data = response.status, dict(response.getheaders()), response.read().decode()
    finally:
      connection.close()
    return status, response_headers, data

  def initialize(self, version='2025-11-25'):
    status, headers, data = self.request({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize',
                                         'params': {'protocolVersion': version, 'capabilities': {}, 'clientInfo': {'name': 'test', 'version': '1'}}})
    self.assertEqual(status, 200)
    self.sid = headers['MCP-Session-Id']
    self.assertEqual(json.loads(data)['result']['protocolVersion'], '2025-11-25')
    self.assertEqual(self.request({'jsonrpc': '2.0', 'method': 'notifications/initialized'})[0], 202)
    return json.loads(data)['result']

  def rpc(self, method, params=None, req_id=2):
    return {'jsonrpc': '2.0', 'id': req_id, 'method': method, 'params': params or {}}

  def test_lifecycle_tools_and_version_negotiation(self):
    self.initialize('2099-01-01')
    status, _, data = self.request(self.rpc('tools/list'))
    self.assertEqual(status, 200)
    self.assertEqual([t['name'] for t in json.loads(data)['result']['tools']],
                     ['scan_vehicle', 'wait_for_scan', 'get_scan_status', 'get_scan_report', 'get_scan_evidence'])
    self.assertEqual(self.request(self.rpc('ping'))[0], 200)
    self.assertEqual(self.request(method='GET')[0], 405)
    self.assertEqual(self.manager.calls, 0)

  def test_saved_reports_use_the_advertised_server_version(self):
    version = self.initialize()['serverInfo']['version']
    self.manager.store.save('a' * 32, evidence())
    bundle = self.manager.store.get()
    self.assertEqual(bundle['server_version'], version)
    self.assertEqual(bundle['report']['server_version'], version)
    self.assertEqual(bundle['evidence']['server_version'], version)
    for name, extra in (('get_scan_report', {}), ('get_scan_evidence', {}), ('get_scan_evidence', {'raw': True})):
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': extra}))
      result = json.loads(data)['result']
      self.assertFalse(result['isError'], result)
      self.assertEqual(result['structuredContent']['server_version'], version)
    self.assertEqual(self.manager.calls, 0)

  def test_incompatible_history_is_filtered_without_starting_a_scan(self):
    version = self.initialize()['serverInfo']['version']
    legacy_id = 'b' * 32
    legacy_path = Path(self.tmp.name) / f'{legacy_id}.json'
    legacy_data = json.dumps({'server_version': '2.0.0', 'report': {'old_fault_marker': True}, 'evidence': {}})
    legacy_path.write_text(legacy_data)
    reads = [('get_scan_report', {}), ('get_scan_evidence', {}), ('get_scan_evidence', {'raw': True}), ('get_scan_status', {})]
    for name, extra in reads:
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': {'scan_id': 'latest', **extra}}))
      result = json.loads(data)['result']
      self.assertTrue(result['isError'])
      self.assertIn('No compatible saved reports', result['structuredContent']['error'])
      self.assertIn('explicit request', result['structuredContent']['error'])
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': {'scan_id': legacy_id, **extra}}))
      self.assertTrue(json.loads(data)['result']['isError'])
      self.assertIn('Incompatible report', data)

    self.manager.store.save('a' * 32, evidence())
    os.utime(Path(self.tmp.name) / f'{"a" * 32}.json', (1, 1))
    for name, extra in reads:
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': {'scan_id': 'latest', **extra}}))
      result = json.loads(data)['result']
      self.assertFalse(result['isError'], result)
      self.assertEqual(result['structuredContent']['scan_id'], 'a' * 32)
      if name != 'get_scan_status':
        self.assertEqual(result['structuredContent']['server_version'], version)
      self.assertNotIn('old_fault_marker', data)
    self.assertEqual(legacy_path.read_text(), legacy_data)
    self.assertEqual(self.manager.calls, 0)
    self.assertFalse(self.server.jobs)

  def test_async_scan_status_and_evidence_without_rescan(self):
    self.initialize()
    status, headers, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    self.assertEqual(status, 200)
    self.assertEqual(headers['Content-Type'], 'application/json')
    scan_id = json.loads(data)['result']['structuredContent']['scan_id']
    self.assertTrue(self.server.jobs[(self.sid, 2)].done.wait(4))
    _, _, body = self.request(self.rpc('tools/call', {'name': 'get_scan_status', 'arguments': {'scan_id': scan_id}}, 4))
    lifecycle = json.loads(body)['result']['structuredContent']
    self.assertEqual(lifecycle['execution'], 'finished')
    self.assertEqual(lifecycle['restoration']['state'], 'verified')
    _, _, body = self.request(self.rpc('tools/call', {'name': 'get_scan_evidence', 'arguments': {'scan_id': scan_id}}, 3))
    self.assertEqual(json.loads(body)['result']['structuredContent']['scan_id'], scan_id)
    self.assertEqual(self.manager.calls, 1)
    self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'wait': False}}, 2))
    self.assertEqual(self.manager.calls, 1)  # same session/request ID is replayed

  def test_default_scan_waits_without_a_progress_token(self):
    self.initialize()
    _, headers, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715'}}))
    self.assertEqual(headers['Content-Type'], 'text/event-stream')
    messages = [json.loads(line[6:]) for line in data.splitlines() if line.startswith('data: ')]
    self.assertEqual(len(messages), 1)
    self.assertEqual(messages[0]['result']['structuredContent']['execution'], 'finished')
    self.assertEqual(messages[0]['result']['structuredContent']['restoration']['state'], 'verified')
    args = self.server.jobs[(self.sid, 2)].result
    self.assertEqual([(r['bus'], r['obd_multiplexing']) for r in args['routes']], [(1, True)])
    self.assertTrue(args['cache']['fast_requested'])

  def stream_result(self, data):
    messages = [json.loads(line[6:]) for line in data.splitlines() if line.startswith('data: ')]
    return messages[-1]['result']

  def test_bounded_wait_returns_progress_and_can_be_resumed_without_rescan(self):
    self.initialize()
    self.manager.block = True
    with patch('openpilot.selfdrive.diagnostics.mcp.WAIT_SECONDS', 0.1, create=True):
      start = time.monotonic()
      _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715'}}), timeout=0.6)
      self.assertLess(time.monotonic() - start, 0.6)
      result = self.stream_result(data)
      self.assertFalse(result['isError'], result)
      state = result['structuredContent']
      scan_id = state['scan_id']
      self.assertEqual(state['execution'], 'running')
      self.assertFalse(state['report_ready'])
      self.assertTrue(state['wait_expired'])
      self.assertEqual(state['next_tool'], 'wait_for_scan')
      self.assertEqual(state['next_arguments'], {'scan_id': scan_id})
      self.assertIn('message', state['progress'])
      self.assertIn('seconds_since_update', state)
      self.assertTrue(self.manager.started.wait(2))
      # A late client timeout/cancellation must not cancel the background scan.
      self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 2}})
      for request_id in (3, 4):
        _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': scan_id}}, request_id), timeout=0.6)
        result = self.stream_result(data)
        self.assertFalse(result['isError'], result)
        self.assertEqual(result['structuredContent']['scan_id'], scan_id)
        self.assertTrue(result['structuredContent']['wait_expired'])
        self.assertFalse(self.server.jobs[(self.sid, 2)].cancel.is_set())
      self.assertEqual(self.manager.calls, 1)

  def test_wait_reports_restoration_then_returns_findings_when_restored(self):
    self.initialize()
    restore = threading.Event()
    self.addCleanup(restore.set)
    self.manager.release.set()
    self.manager.transport_factory = lambda cancel: ParkedPanda(cancel, self.manager.started, self.manager.release, restore)
    _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    scan_id = json.loads(data)['result']['structuredContent']['scan_id']
    with patch('openpilot.selfdrive.diagnostics.mcp.WAIT_SECONDS', 0.1):
      deadline = time.monotonic() + 3
      while self.manager.get_status(scan_id)['phase'] != 'restoring' and time.monotonic() < deadline:
        time.sleep(0.01)
      _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': scan_id}}, 3))
      state = self.stream_result(data)['structuredContent']
      self.assertEqual(state['restoration']['state'], 'in_progress')
      self.assertEqual(state['execution'], 'running')
      self.assertFalse(state['report_ready'])
      self.assertEqual(state['next_tool'], 'wait_for_scan')
    # Completion wakes a pending read, not another full wait window.
    with patch('openpilot.selfdrive.diagnostics.mcp.WAIT_SECONDS', 2):
      connection, response = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': scan_id}}, 4), read=False)
      start = time.monotonic()
      restore.set()
      result = self.stream_result(response.read().decode())
      connection.close()
      self.assertLess(time.monotonic() - start, 1)
      self.assertFalse(result['isError'], result)
      self.assertEqual(result['structuredContent']['scan_id'], scan_id)
      self.assertEqual(result['structuredContent']['execution'], 'finished')
      self.assertEqual(result['structuredContent']['restoration']['state'], 'verified')
      self.assertIn('items', result['structuredContent'])

  def test_readonly_wait_cancellation_does_not_cancel_the_scan(self):
    self.initialize()
    self.manager.block = True
    self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    job = self.server.jobs[(self.sid, 2)]
    self.assertTrue(self.manager.started.wait(1))
    # Another session can observe it, but cancelling that read cannot mutate it.
    self.sid = None
    self.initialize()
    connection, response = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': job.scan_id}}, 3), read=False)
    self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 3}})
    result = self.stream_result(response.read().decode())
    connection.close()
    self.assertTrue(result['structuredContent']['wait_cancelled'])
    self.assertFalse(job.cancel.is_set())
    self.assertEqual(self.manager.calls, 1)

  def test_wait_for_saved_report_idle_unknown_and_interrupted_are_terminal(self):
    self.initialize()
    _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan'}))
    self.assertEqual(self.stream_result(data)['structuredContent']['execution'], 'idle')
    self.manager.store.save('a' * 32, evidence(execution='finished', restoration={'state': 'verified'}))
    _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': 'a' * 32}}))
    result = self.stream_result(data)
    self.assertFalse(result['isError'], result)
    self.assertEqual(result['structuredContent']['scan_id'], 'a' * 32)
    self.assertNotEqual(result['structuredContent'].get('next_tool'), 'wait_for_scan')
    _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': 'f' * 32}}))
    self.assertTrue(json.loads(data)['result']['isError'])
    self.assertNotIn(self.tmp.name, data)
    # Persist a worker's running state but release its lock, just as after a crash.
    operation = self.manager._reserve()
    operation.lockfile.close()
    self.manager.lock.release()
    self.manager.operation = None
    _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan'}))
    result = self.stream_result(data)
    self.assertTrue(result['isError'])
    self.assertEqual(result['structuredContent']['execution'], 'interrupted')
    self.assertEqual(result['structuredContent']['restoration']['state'], 'unknown')
    self.assertIsNone(result['structuredContent']['next_tool'])
    self.assertEqual(self.manager.calls, 0)

  def test_active_wait_pins_the_original_id_and_observes_persisted_progress(self):
    self.initialize()
    self.manager.block = True
    self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    scan_id = self.server.jobs[(self.sid, 2)].scan_id
    self.assertTrue(self.manager.started.wait(2))
    observer = DiagnosticManager(self.manager.store)  # no in-memory operation; observe persisted state
    self.server.manager = observer
    with patch.object(observer, 'get_status', wraps=observer.get_status) as status_read, \
         patch('openpilot.selfdrive.diagnostics.mcp.WAIT_SECONDS', 0.1):
      _, _, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan'}, 3))
      result = self.stream_result(data)
      self.assertEqual(result['structuredContent']['scan_id'], scan_id)
      self.assertEqual(status_read.call_args_list[0].args, ('active',))
      self.assertTrue(all(call.args == (scan_id,) for call in status_read.call_args_list[1:]))
    self.assertEqual(self.manager.calls, 1)

  def test_wait_status_read_failure_finishes_sse_without_claiming_recovery(self):
    self.initialize()
    self.manager.block = True
    self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    scan_id = self.server.jobs[(self.sid, 2)].scan_id
    status = self.manager.get_status(scan_id)
    with patch.object(self.manager, 'get_status', side_effect=[status, OSError('internal filesystem path')]):
      _, headers, data = self.request(self.rpc('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': scan_id}}, 3))
      self.assertEqual(headers['Content-Type'], 'text/event-stream')
      result = self.stream_result(data)
      self.assertTrue(result['isError'])
      self.assertEqual(result['structuredContent']['execution'], 'interrupted')
      self.assertEqual(result['structuredContent']['restoration']['state'], 'unknown')
      self.assertIsNone(result['structuredContent']['next_tool'])
      self.assertNotIn('internal filesystem path', data)
      self.assertNotIn('HTTP/1.1', data)

  def test_scan_schema_is_fresh_obd_only_and_rejects_stale_mode_arguments(self):
    self.initialize()
    _, _, data = self.request(self.rpc('tools/list'))
    properties = json.loads(data)['result']['tools'][0]['inputSchema']['properties']
    self.assertEqual(set(properties), {'target', 'details', 'wait'})
    self.assertTrue(properties['wait']['default'])
    for args in ({'broad': True}, {'fast': True}, {'broad': True, 'fast': True, 'details': True}):
      _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': args}))
      self.assertIn('error', json.loads(data))
    self.assertEqual(self.manager.calls, 0)

  def test_report_reads_while_running_return_status_without_filesystem_errors(self):
    self.initialize()
    self.manager.block = True
    _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}))
    scan_id = json.loads(data)['result']['structuredContent']['scan_id']
    self.assertTrue(self.manager.started.wait(1))
    for name, extra in (('get_scan_report', {}), ('get_scan_evidence', {}), ('get_scan_evidence', {'raw': True})):
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': {'scan_id': scan_id, **extra}}, 3))
      result = json.loads(data)['result']
      self.assertFalse(result['isError'], result)
      state = result['structuredContent']
      self.assertEqual(state['scan_id'], scan_id)
      self.assertEqual(state['execution'], 'running')
      self.assertFalse(state['report_ready'])
      self.assertEqual(state['next_tool'], 'wait_for_scan')
      self.assertNotIn(self.tmp.name, data)
    self.assertEqual(self.manager.calls, 1)

  def test_missing_report_has_a_friendly_error(self):
    self.initialize()
    for name in ('get_scan_report', 'get_scan_evidence'):
      _, _, data = self.request(self.rpc('tools/call', {'name': name, 'arguments': {'scan_id': 'f' * 32}}))
      result = json.loads(data)['result']
      self.assertTrue(result['isError'])
      self.assertNotIn(self.tmp.name, data)
      self.assertIn('Unknown scan', result['structuredContent']['error'])
    self.assertEqual(self.manager.calls, 0)

  def test_cancel_and_busy(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'wait': True}}, 10), read=False)
    self.assertTrue(self.manager.started.wait(1))
    _, _, busy = self.request(self.rpc('tools/call', {'name': 'scan_vehicle'}, 11))
    self.assertTrue(json.loads(busy)['result']['isError'])
    self.assertEqual(json.loads(busy)['result']['structuredContent']['active_scan']['scan_id'], self.server.jobs[(self.sid, 10)].scan_id)
    self.assertEqual(self.request({'jsonrpc': '2.0', 'method': 'notifications/cancelled', 'params': {'requestId': 10}})[0], 202)
    self.assertTrue(self.manager.cancelled.wait(1))
    response.read()
    connection.close()

  def test_cancellation_does_not_cross_sessions(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'wait': True}}, 10), read=False)
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
    for args in ({'wait': 'yes'}, {'target': '0x7df'}, {'details': 1}):
      _, _, data = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': args}))
      self.assertTrue(json.loads(data)['result']['isError'])
    self.assertEqual(self.manager.calls, 0)

  def test_delete_session_cancels_owned_work(self):
    self.initialize()
    self.manager.block = True
    connection, response = self.request(self.rpc('tools/call', {'name': 'scan_vehicle', 'arguments': {'wait': True}}, 10), read=False)
    self.assertEqual(self.request(method='DELETE')[0], 200)
    self.assertTrue(self.manager.cancelled.wait(1))
    response.read()
    connection.close()
    self.assertEqual(self.request(self.rpc('ping'))[0], 404)

  def test_non_loopback_bind_refused(self):
    with self.assertRaises(ValueError):
      MCPServer(('0.0.0.0', 0), manager=self.manager)
