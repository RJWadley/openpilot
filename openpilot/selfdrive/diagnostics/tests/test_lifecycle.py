"""Public scan lifecycle with a simulated CAN adapter and real scanner/storage."""
import tempfile
import threading
import time
import unittest
import multiprocessing

from openpilot.selfdrive.diagnostics.manager import DiagnosticManager, ReportStore, scan_args
from tools.scripts.car import diagnose
from tools.scripts.car.tests.test_diagnose import FakePanda
from opendbc.car.structs import CarParams
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence


class ParkedPanda(FakePanda):
  def __init__(self, cancel, started, release, restore=None, fail_restore=False):
    super().__init__({diagnose.Target(0x715, 0x77f): {b'\x19\x02\xff': bytes.fromhex('5902ff90161488')}})
    self.cancel, self.started, self.release = cancel, started, release
    self.restore, self.fail_restore = restore, fail_restore
    self.acquired = False

  def health(self):
    self.started.set()
    if not self.release.wait(5):
      raise RuntimeError('Test gate timed out')
    if self.cancel.is_set():
      raise RuntimeError('Diagnostic scan cancelled')
    return super().health()

  def set_safety_mode(self, *args):
    self.acquired = True
    return super().set_safety_mode(*args)

  def close(self):
    if self.restore is not None:
      self.restore.wait(5)
    if self.fail_restore:
      raise RuntimeError('Coordinator did not return to idle')
    super().close()


def interrupted_worker(root, started, release):
  manager = DiagnosticManager(ReportStore(root), lambda cancel: ParkedPanda(cancel, started, release))
  manager.scan(scan_args('0x715'))


class TestLifecycle(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.started, self.release = threading.Event(), threading.Event()
    self.addCleanup(self.release.set)
    self.store = ReportStore(self.tmp.name)
    self.manager = DiagnosticManager(self.store, lambda cancel: ParkedPanda(cancel, self.started, self.release))
    self.args = scan_args('0x715')
    self.args.rx_addr, self.args.timeout, self.args.probe_timeout = 0x77f, 0.01, 0.001

  def tearDown(self):
    self.release.set()
    if self.manager.operation is not None:
      self.manager.operation.cancel.set()
      self.manager.operation.done.wait(6)

  def test_scan_returns_id_before_hardware_finishes_and_status_survives_restart(self):
    operation = self.manager.start(self.args)
    self.assertTrue(self.started.wait(3))
    status = self.manager.get_status(operation.scan_id)
    self.assertEqual(status['execution'], 'running')
    self.assertEqual(status['phase'], 'preflight')
    self.assertFalse(status['report_ready'])
    with self.assertRaises(RuntimeError) as busy:
      self.manager.start(self.args)
    self.assertEqual(busy.exception.status['scan_id'], operation.scan_id)
    self.release.set()
    self.assertTrue(operation.done.wait(5))
    status = DiagnosticManager(ReportStore(self.tmp.name)).get_status(operation.scan_id)
    self.assertEqual(status['execution'], 'finished')
    self.assertEqual(status['coverage'], 'partial')
    self.assertEqual(status['restoration']['state'], 'verified')
    self.assertTrue(status['report_ready'])
    self.assertIsNotNone(status['completed_at'])

  def test_collection_can_finish_while_restoration_is_pending_or_unverified(self):
    restore = threading.Event()
    self.addCleanup(restore.set)
    self.manager = DiagnosticManager(self.store, lambda cancel: ParkedPanda(cancel, self.started, self.release, restore, True))
    self.release.set()
    operation = self.manager.start(self.args)
    observer = DiagnosticManager(ReportStore(self.tmp.name))
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
      status = observer.get_status(operation.scan_id)
      if status['phase'] == 'restoring':
        break
      time.sleep(0.01)
    self.assertEqual(status['restoration']['state'], 'in_progress')
    self.assertIsNotNone(status['collection_finished_at'])
    self.assertIn('normal openpilot operation', status['progress']['message'])
    restore.set()
    self.assertTrue(operation.done.wait(3))
    status = observer.get_status(operation.scan_id)
    self.assertEqual(status['execution'], 'finished')
    self.assertEqual(status['restoration']['state'], 'unverified')
    self.assertTrue(status['report_ready'])

  def test_progress_counts_real_probes_and_reads_and_uses_reported_identity(self):
    target = diagnose.Target(0x715, 0x77f)
    panda = FakePanda({target: {b'\x19\x02\xff': bytes.fromhex('5902ff90161488'),
                                b'\x22\xf1\x97': b'\x62\xf1\x97AirbagVW20'}})
    self.args.rx_addr = None
    updates = []
    result = diagnose.scan(panda, self.args, {}, {}, CarParams.SafetyModel, progress=updates.append)
    self.assertEqual(result['status'], 'partial')
    discovery = [p for p in updates if p['phase'] == 'discovering']
    self.assertEqual((discovery[0]['current'], discovery[0]['total']), (0, 1))
    self.assertEqual((discovery[-1]['current'], discovery[-1]['total']), (1, 1))
    reading = [p for p in updates if p['phase'] == 'reading']
    self.assertEqual(reading[-1]['current'], 1)
    self.assertEqual(reading[-1]['total'], 1)
    self.assertTrue(any('AirbagVW20' in p['message'] and '0x715' in p['message'] for p in reading))
    self.assertEqual(updates[-1]['phase'], 'collection_complete')

  def test_worker_crash_is_interrupted_not_restored_and_old_report_is_identified(self):
    self.store.save('e' * 32, evidence(started_at='2020-01-01T00:00:00+00:00'))
    context = multiprocessing.get_context('spawn')
    started, release = context.Event(), context.Event()
    process = context.Process(target=interrupted_worker, args=(self.tmp.name, started, release))
    process.start()
    try:
      self.assertTrue(started.wait(4))
      status = self.manager.get_status()
      self.assertEqual(status['execution'], 'running')
      page = self.store.get_report()
      self.assertEqual(page['active_scan_id'], status['scan_id'])
      self.assertFalse(page['is_active_scan'])
      self.assertGreater(page['report_age_seconds'], 86400)
      with self.assertRaises(RuntimeError) as busy:
        self.manager.start(self.args)
      self.assertEqual(busy.exception.status['scan_id'], status['scan_id'])
      process.terminate()
      process.join(3)
      after = DiagnosticManager(ReportStore(self.tmp.name)).get_status(status['scan_id'])
      self.assertEqual(after['execution'], 'interrupted')
      self.assertEqual(after['restoration']['state'], 'unknown')
      self.assertIsNone(after['active_scan_id'])
      self.assertFalse(after['report_ready'])
    finally:
      if process.is_alive():
        process.terminate()
      process.join(3)

  def test_report_save_failure_still_restores_and_reports_failure(self):
    self.store.byte_limit = 1
    self.release.set()
    operation = self.manager.start(self.args)
    self.assertTrue(operation.done.wait(3))
    status = self.manager.get_status(operation.scan_id)
    self.assertEqual(status['execution'], 'failed')
    self.assertEqual(status['restoration']['state'], 'verified')
    self.assertFalse(status['report_ready'])
    self.assertIn('storage limit', status['error'])

  def test_cancel_before_acquisition_is_not_coverage_or_restoration_failure(self):
    operation = self.manager.start(self.args)
    self.assertTrue(self.started.wait(3))
    operation.cancel.set()
    self.release.set()
    self.assertTrue(operation.done.wait(3))
    state = self.manager.get_status(operation.scan_id)
    self.assertEqual(state['execution'], 'cancelled')
    self.assertEqual(state['coverage'], 'unavailable')
    self.assertEqual(state['restoration']['state'], 'not_needed')
    page = self.store.get_report(operation.scan_id)
    self.assertEqual(page['execution'], 'cancelled')
    self.assertEqual(page['coverage'], 'unavailable')
