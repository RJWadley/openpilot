import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.diagnostics.interlock import DiagnosticInterlock
from openpilot.selfdrive.diagnostics.manager import DiagnosticManager, ReportStore, scan_args
from tools.scripts.car import diagnose


def evidence(**kwargs):
  return {'schema_version': 4, 'report_kind': 'technical_evidence', 'status': 'partial', 'vehicle_coverage_complete': False,
          'ecus': [], 'errors': [], 'warnings': [], **kwargs}


class TestStore(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.store = ReportStore(self.tmp.name)

  def test_roundtrip_and_latest_survive_restart(self):
    report = self.store.save('a' * 32, evidence(raw_marker='preserved'))
    self.assertEqual(report['scan_id'], 'a' * 32)
    self.assertNotIn('raw_marker', report)
    read = ReportStore(self.tmp.name).get('latest')
    self.assertEqual(read['report'], report)
    self.assertEqual(read['evidence']['raw_marker'], 'preserved')

  def test_exclusive_and_path_validation(self):
    self.store.save('a' * 32, evidence())
    with self.assertRaises(FileExistsError):
      self.store.save('a' * 32, evidence())
    for name in ('../private', '/etc/passwd', '', 'z' * 32):
      with self.assertRaises(ValueError):
        self.store.get(name)
      with self.assertRaises(ValueError):
        self.store.save(name, evidence())

  def test_retention_and_size_cap_only_remove_owned_reports(self):
    unrelated = Path(self.tmp.name) / 'keep.json'
    unrelated.write_text('user data')
    self.store.count = 2
    for n in range(4):
      self.store.save(f'{n:032x}', evidence())
    self.assertEqual(len(self.store._files()), 2)
    self.assertEqual(unrelated.read_text(), 'user data')
    self.store.byte_limit = 1
    with self.assertRaises(ValueError):
      self.store.save('f' * 32, evidence())
    self.assertEqual(len(self.store._files()), 2)

  def test_recovery_failure_is_present_in_saved_readable_report(self):
    report = self.store.save('b' * 32, evidence(recovery_required=True))
    self.assertTrue(report['recovery_required'])
    self.assertTrue(self.store.get()['report']['recovery_required'])

  def test_byte_limit_prunes_oldest_reports(self):
    self.store.save('0' * 32, evidence())
    self.store.byte_limit = self.store._files()[0].stat().st_size * 2 + 10
    for n in range(1, 4):
      self.store.save(f'{n:032x}', evidence())
    self.assertEqual(len(self.store._files()), 2)
    self.assertLessEqual(sum(p.stat().st_size for p in self.store._files()), self.store.byte_limit)
    self.assertEqual(self.store.get()['report']['scan_id'], f'{3:032x}')

  def test_symlink_refused(self):
    source = Path(self.tmp.name) / 'external.json'
    source.write_text('{}')
    (Path(self.tmp.name) / ('a' * 32 + '.json')).symlink_to(source)
    with self.assertRaises(ValueError):
      self.store.get('a' * 32)


class TestManager(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.closed = []
    self.transport = SimpleNamespace(close=lambda: self.closed.append(True))
    self.manager = DiagnosticManager(ReportStore(self.tmp.name), lambda cancel: self.transport)

  def run_scan(self, result=None, error=None):
    with patch.object(diagnose, 'load_dataset', return_value={}), patch.object(diagnose, 'load_known_targets', return_value={}), \
         patch.object(diagnose, 'scan', return_value=result or evidence(), side_effect=error):
      return self.manager.scan(scan_args())

  def test_scan_closes_and_saves(self):
    report = self.run_scan()
    self.assertEqual(self.closed, [True])
    self.assertEqual(self.manager.store.get()['report'], report)

  def test_report_publication_racing_with_a_read_retries_after_ready(self):
    report = self.run_scan()
    page = self.manager.store.get_report(report['scan_id'])
    with patch.object(self.manager.store, 'get_evidence', side_effect=[FileNotFoundError(), page]) as read:
      self.assertEqual(self.manager.get_report(report['scan_id']), page)
    self.assertEqual(read.call_count, 2)

  def test_exception_still_closes_and_saves_failure(self):
    report = self.run_scan(error=RuntimeError('wire failed'))
    self.assertEqual(self.closed, [True])
    self.assertTrue(report['setup_error'])
    self.assertIn('wire failed', report['errors'][0])

  def test_cleanup_failure_never_claims_recovered(self):
    def fail():
      raise RuntimeError('still locked')
    self.transport.close = fail
    report = self.run_scan()
    self.assertTrue(report['recovery_required'])
    self.assertIn('still locked', report['errors'][0])

  def test_busy_does_not_call_transport(self):
    self.manager.lock.acquire()
    try:
      with self.assertRaisesRegex(RuntimeError, 'already running'):
        self.manager.scan(scan_args())
    finally:
      self.manager.lock.release()
    self.assertEqual(self.closed, [])

  def test_input_validation(self):
    self.assertIsNone(scan_args().addr)
    self.assertEqual(scan_args('0x715').addr, 0x715)
    for kw in ({'target': '0x7df'}, {'target': 123}, {'broad': 1}, {'fast': 'yes'}, {'details': None}):
      with self.subTest(kw=kw), self.assertRaises(ValueError):
        scan_args(**kw)

  def test_cli_defaults_to_coordinated_engine(self):
    import contextlib
    import io
    report = self.manager.store.save('c' * 32, evidence())
    with patch('openpilot.selfdrive.diagnostics.manager.DiagnosticManager', return_value=self.manager), \
         patch.object(self.manager, 'scan', return_value=report), patch.object(diagnose, 'check_pandad') as direct_check, \
         contextlib.redirect_stdout(io.StringIO()) as output:
      diagnose.main(['--json'])
    direct_check.assert_not_called()
    self.assertEqual(json.loads(output.getvalue())['scan_id'], 'c' * 32)


class TestInterlock(unittest.TestCase):
  def test_latches_across_missing_or_invalid_messages(self):
    interlock = DiagnosticInterlock()
    class SM(dict):
      updated = {'diagnosticState': True}
      valid = {'diagnosticState': True}
      logMonoTime = {'diagnosticState': time.monotonic_ns()}
    sm = SM(diagnosticState=SimpleNamespace(phase='preparing', sessionId=9, route=1))
    interlock.update(sm)
    self.assertTrue(interlock.blocked)
    self.assertTrue(interlock.pause_controls)
    sm.updated['diagnosticState'] = False
    sm['diagnosticState'].phase = 'idle'
    interlock.update(sm)
    self.assertTrue(interlock.blocked)
    sm.updated['diagnosticState'] = True
    sm.valid['diagnosticState'] = False
    interlock.update(sm)
    self.assertTrue(interlock.blocked)
    sm.valid['diagnosticState'] = True
    sm['diagnosticState'].phase = 'restoring'
    interlock.update(sm)
    self.assertTrue(interlock.blocked)
    self.assertFalse(interlock.pause_controls)  # permit ordinary CI.init recovery
    sm['diagnosticState'].phase = 'idle'
    stamp = sm.logMonoTime['diagnosticState']
    sm.logMonoTime['diagnosticState'] = stamp - 2_000_000_000
    interlock.update(sm)
    self.assertTrue(interlock.blocked)
    sm.logMonoTime['diagnosticState'] = time.monotonic_ns()
    interlock.update(sm)
    self.assertFalse(interlock.blocked)

  def test_restart_with_recovery_flag_starts_locked(self):
    self.assertTrue(DiagnosticInterlock(recovery_required=True).blocked)
