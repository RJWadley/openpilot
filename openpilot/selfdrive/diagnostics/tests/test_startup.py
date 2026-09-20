"""Boot policy plus the real manager/scanner/cache lifecycle, with simulated CAN."""
import http.client
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.selfdrive.diagnostics.manager import BusyError, DiagnosticManager, ReportStore, scan_args
from openpilot.selfdrive.diagnostics.mcp import MCPServer
from openpilot.selfdrive.diagnostics.startup import StartupPreparation, start_preparation
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence
from tools.scripts.car import diagnose
from tools.scripts.car.tests import test_diagnose


class TestBootPolicy(unittest.TestCase):
  boot = '11111111-1111-1111-1111-111111111111'

  def setUp(self):
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    self.marker = Path(tmp.name) / 'boot.json'
    self.manager = SimpleNamespace(operation=None, prepare=Mock())
    self.startup = StartupPreparation(self.manager, self.boot, self.marker)

  def tick(self, now, *, ready=True, phase='idle', valid=True, age=0):
    self.startup.tick(SimpleNamespace(ready=ready, phase=phase), valid=valid, stamp=now - age, now=now)

  def test_requires_fresh_stable_native_readiness(self):
    for kwargs in ({'valid': False}, {'age': 2}, {'age': -1}, {'ready': False}, {'phase': 'restoring'}):
      self.tick(10, **kwargs)
      self.tick(20, **kwargs)
    self.manager.prepare.assert_not_called()
    self.tick(30)
    self.tick(31)
    self.tick(32, ready=False)
    self.tick(33)
    self.tick(35)
    self.manager.prepare.assert_called_once()

  def test_same_boot_restarts_and_recovery_cycles_do_not_repeat_preparation(self):
    self.tick(10)
    self.tick(12)
    self.tick(20, ready=False)
    self.tick(30)
    self.startup = StartupPreparation(self.manager, self.boot, self.marker)
    self.tick(40)
    self.tick(42)
    self.manager.prepare.assert_called_once()
    self.startup = StartupPreparation(self.manager, '22222222-2222-2222-2222-222222222222', self.marker)
    self.tick(50)
    self.tick(52)
    self.assertEqual(self.manager.prepare.call_count, 2)

  def test_explicit_request_or_competing_cli_does_not_trigger_another_automatic_scan(self):
    self.manager.operation = object()
    self.tick(10, ready=False)
    self.manager.prepare.assert_not_called()
    self.manager.operation = None
    self.tick(20)
    self.tick(22)
    self.manager.prepare.assert_not_called()
    self.startup = StartupPreparation(self.manager, '22222222-2222-2222-2222-222222222222', self.marker)
    self.manager.prepare.side_effect = BusyError({'execution': 'running'})
    self.tick(30)
    self.tick(32)
    self.tick(34)
    self.manager.prepare.assert_called_once()

  def test_marker_is_written_before_start_and_failure_does_not_retry(self):
    def prepare():
      self.assertTrue(StartupPreparation(self.manager, self.boot, self.marker).attempted)
      raise RuntimeError('wire unavailable')
    self.manager.prepare.side_effect = prepare
    self.tick(10)
    with self.assertRaises(RuntimeError):
      self.tick(12)
    self.tick(14)
    self.manager.prepare.assert_called_once()

  def test_failed_marker_write_and_shutdown_never_start_preparation(self):
    self.tick(10)
    with patch.object(diagnose, 'save_module_cache', side_effect=OSError('disk full')), self.assertRaises(OSError):
      self.tick(12)
    self.manager.prepare.assert_not_called()
    self.startup.close()
    self.tick(14)
    self.manager.prepare.assert_not_called()

  def test_local_server_does_not_autoprepare(self):
    with patch.object(Path, 'is_file', return_value=False):
      self.assertIsNone(start_preparation(self.manager))
    self.manager.prepare.assert_not_called()


class InventoryPanda(test_diagnose.FakePanda):
  def __init__(self, cancel, started, release, restore=None, fail_restore=False):
    super().__init__(test_diagnose.TestRoutesAndCache().ecus())
    self.cancel, self.started, self.release = cancel, started, release
    self.restore, self.fail_restore = restore, fail_restore
    self.acquired = False
    self.display = []

  def health(self):
    self.started.set()
    if not self.release.wait(5) or self.cancel.is_set():
      raise RuntimeError('Preparation cancelled')
    return super().health()

  def set_safety_mode(self, *args):
    self.acquired = True
    super().set_safety_mode(*args)

  def set_preparing(self, preparing):
    self.display.append(preparing)

  def close(self):
    if self.restore is not None and not self.restore.wait(5):
      raise RuntimeError('test restoration gate expired')
    super().close()
    if self.fail_restore:
      raise RuntimeError('Recovery not verified')


class TestInventoryLifecycle(unittest.TestCase):
  def setUp(self):
    tmp = tempfile.TemporaryDirectory()
    self.addCleanup(tmp.cleanup)
    self.root = Path(tmp.name)
    self.enterContext(patch.object(diagnose, 'module_cache_path', return_value=self.root / 'modules.json'))
    self.enterContext(patch.object(diagnose, 'generic_probes', return_value={(0x715, None), (0x7e0, None)}))
    self.enterContext(patch.object(diagnose, 'load_known_targets', return_value={}))
    self.enterContext(patch.object(diagnose, 'load_dataset', return_value={}))
    self.started, self.release, self.restore = threading.Event(), threading.Event(), threading.Event()
    self.restore.set()
    self.pandas = []
    self.fail_restore = False
    self.manager = DiagnosticManager(ReportStore(self.root / 'reports'), self.factory)
    self.args = scan_args()
    self.args.timeout, self.args.probe_timeout, self.args.scan_timeout = 0.01, 0.001, 5

  def factory(self, cancel):
    panda = InventoryPanda(cancel, self.started, self.release, self.restore, self.fail_restore)
    self.pandas.append(panda)
    return panda

  def tearDown(self):
    self.release.set()
    self.restore.set()
    if self.manager.operation is not None:
      self.manager.operation.cancel.set()
      self.assertTrue(self.manager.operation.done.wait(6))

  def finish(self, operation):
    self.release.set()
    self.assertTrue(operation.done.wait(5))
    return self.manager.get_status(operation.scan_id)

  def test_preparation_survives_restart_and_never_replaces_latest_fault_report(self):
    self.manager.store.save('a' * 32, evidence())
    state = self.finish(self.manager.prepare(self.args))
    self.assertEqual(state['kind'], 'preparation')
    self.assertFalse(state['scan_requested'])
    self.assertFalse(state['report_ready'])
    self.assertTrue(state['inventory_ready'])
    self.assertEqual(state['restoration']['state'], 'verified')
    self.assertEqual(self.manager.store.get()['report']['scan_id'], 'a' * 32)
    self.assertFalse(any(req[0] in (3, 7, 10, 0x19) for _, req in self.pandas[0].requests))
    self.assertTrue(all(self.pandas[0].display))
    self.manager = DiagnosticManager(ReportStore(self.root / 'reports'), self.factory)
    with patch.object(diagnose, 'discover', side_effect=AssertionError('Must reuse persisted inventory')):
      result = self.manager.scan(self.args)
    self.assertEqual(result['cache']['routes_reused'], 1)
    self.assertTrue(any(req[0] == 0x19 for _, req in self.pandas[1].requests))
    self.assertIn(False, self.pandas[1].display)

  def test_scan_joins_preparation_then_reads_fresh_faults_without_duplicate_discovery(self):
    with patch.object(diagnose, 'discover', wraps=diagnose.discover) as discover:
      operation = self.manager.prepare(self.args)
      self.assertTrue(self.started.wait(2))
      self.assertIs(self.manager.start(self.args), operation)
      with self.assertRaises(BusyError):
        self.manager.start(self.args)
      state = self.manager.get_status(operation.scan_id)
      self.assertTrue(state['scan_requested'])
      self.assertFalse(state['report_ready'])
      self.assertEqual(state['kind'], 'scan')
      final = self.finish(operation)
    self.assertEqual(discover.call_count, 1)
    self.assertEqual(final['execution'], 'finished')
    self.assertTrue(final['report_ready'])
    self.assertEqual(final['restoration']['state'], 'verified')
    self.assertEqual(len(self.pandas), 2)
    self.assertTrue(all(p.closed for p in self.pandas))
    self.assertEqual(operation.result['cache']['routes_reused'], 1)
    self.assertEqual(operation.result['scan_id'], operation.scan_id)

  def test_request_arriving_during_preparation_restoration_is_not_lost(self):
    self.restore.clear()
    self.release.set()
    operation = self.manager.prepare(self.args)
    deadline = time.monotonic() + 3
    while self.manager.get_status()['phase'] != 'restoring' and time.monotonic() < deadline:
      time.sleep(0.01)
    self.assertEqual(self.manager.get_status()['phase'], 'restoring')
    self.assertIs(self.manager.start(self.args), operation)
    self.restore.set()
    self.assertTrue(self.finish(operation)['report_ready'])

  def test_cancel_or_failed_restoration_never_launches_queued_fault_scan(self):
    for failure in ('cancel', 'restore'):
      with self.subTest(failure=failure):
        self.started.clear()
        self.release.clear()
        self.fail_restore = failure == 'restore'
        count = len(self.pandas)
        operation = self.manager.prepare(self.args)
        self.assertTrue(self.started.wait(2))
        self.manager.start(self.args)
        if failure == 'cancel':
          operation.cancel.set()
        final = self.finish(operation)
        self.assertEqual(len(self.pandas), count + 1)
        self.assertFalse(final['report_ready'])
        self.assertEqual(final['execution'], 'cancelled' if failure == 'cancel' else 'failed')

  def test_finished_preparation_allows_a_new_scan_operation(self):
    operation = self.manager.prepare(self.args)
    self.finish(operation)
    scan = self.manager.start(self.args)
    self.assertNotEqual(scan.scan_id, operation.scan_id)
    self.assertTrue(self.finish(scan)['report_ready'])

  def test_worker_start_failure_does_not_leave_a_joinable_preparation(self):
    with patch.object(threading.Thread, 'start', side_effect=RuntimeError('thread unavailable')), self.assertRaises(RuntimeError):
      self.manager.prepare(self.args)
    self.assertTrue(self.manager.operation.done.is_set())
    self.assertFalse(self.manager.operation.accepting_scan)
    self.assertEqual(self.manager.get_status()['execution'], 'failed')
    self.assertTrue(self.finish(self.manager.start(self.args))['report_ready'])

  def test_mcp_wait_loop_spans_preparation_and_fresh_fault_reading(self):
    server = MCPServer(('127.0.0.1', 0), manager=self.manager)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
    thread.start()
    headers = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}

    def request(method, params, request_id=None):
      connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
      try:
        message = {'jsonrpc': '2.0', 'method': method, 'params': params}
        if request_id is not None:
          message['id'] = request_id
        connection.request('POST', '/mcp', json.dumps(message), headers)
        response = connection.getresponse()
        if response.getheader('MCP-Session-Id'):
          headers.update({'MCP-Session-Id': response.getheader('MCP-Session-Id'), 'MCP-Protocol-Version': '2025-11-25'})
        data = response.read().decode()
        if response.getheader('Content-Type') == 'text/event-stream':
          return [json.loads(line[6:]) for line in data.splitlines() if line.startswith('data: ')][-1]
        return json.loads(data) if data else None
      finally:
        connection.close()

    try:
      request('initialize', {'protocolVersion': '2025-11-25', 'capabilities': {}, 'clientInfo': {'name': 'test', 'version': '1'}}, 1)
      request('notifications/initialized', {})
      with patch.object(diagnose, 'discover', wraps=diagnose.discover) as discover:
        operation = self.manager.prepare(self.args)
        self.assertTrue(self.started.wait(2))
        start = request('tools/call', {'name': 'scan_vehicle', 'arguments': {'target': '0x715', 'wait': False}}, 2)
        self.assertEqual(start['result']['structuredContent']['scan_id'], operation.scan_id)
        with patch('openpilot.selfdrive.diagnostics.mcp.WAIT_SECONDS', 0.05):
          pending = request('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': operation.scan_id}}, 3)
        progress = pending['result']['structuredContent']
        self.assertEqual(progress['next_tool'], 'wait_for_scan')
        self.assertTrue(progress['scan_requested'])
        self.assertFalse(progress['report_ready'])
        self.release.set()
        finished = request('tools/call', {'name': 'wait_for_scan', 'arguments': {'scan_id': operation.scan_id}}, 4)
        self.assertFalse(finished['result']['isError'], finished)
        self.assertEqual(finished['result']['structuredContent']['execution'], 'finished')
        self.assertEqual(finished['result']['structuredContent']['restoration']['state'], 'verified')
        self.assertEqual(discover.call_count, 1)
        self.assertEqual(len(self.pandas), 2)
    finally:
      self.release.set()
      server.cancel_all()
      server.shutdown()
      server.server_close()
      thread.join(2)
