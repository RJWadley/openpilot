import json
import os
import tempfile
import unittest
from pathlib import Path

from openpilot.selfdrive.diagnostics.manager import ReportStore
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence
from openpilot.selfdrive.diagnostics.version import SERVER_VERSION


class TestReports(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.store = ReportStore(self.tmp.name)

  def test_default_report_is_compact_but_keeps_fault_context(self):
    ecu = {'bus': 1, 'obd_multiplexing': True, 'tx_address': '0x715', 'rx_address': '0x77f', 'subaddress': None,
           'identity': {'component': 'AirbagVW20', 'part_number': '5Q0959655J'}, 'dtc_read': True,
           'queries': [{'name': 'uds_codes', 'outcome': 'ok', 'data': [], 'responses': ['ab' * 50000]}],
           'codes': [{'protocol': 'uds', 'code': '0x901614', 'raw_dtc': '901614', 'format': None,
                      'display_code': '9442836 (0x901614)', 'status': ['confirmed'],
                      'search': {'query': '9442836 AirbagVW20 5Q0959655J'},
                      'lookup': {'source': 'OBDex', 'title': 'Reference description', 'entry': {'large': 'x' * 50000}}}]}
    self.store.save('a' * 32, evidence(ecus=[ecu], warnings=['Coverage is incomplete']))
    page = self.store.get_report()
    self.assertLessEqual(len(json.dumps(page).encode()), 8000)
    self.assertNotIn('responses', json.dumps(page))
    fault = next(i['fault'] for i in page['items'] if i['kind'] == 'fault')
    self.assertEqual(fault['raw_dtc'], '901614')
    self.assertEqual(fault['search']['query'], '9442836 AirbagVW20 5Q0959655J')
    self.assertNotIn('entry', fault['lookup'])
    self.assertIn('not a count of active problems', page['interpretation'])
    self.assertFalse(page['vehicle_coverage_complete'])
    self.assertTrue(any(i.get('message') == 'Coverage is incomplete' for i in page['items']))

  def test_large_evidence_is_losslessly_paged_and_ecu_filter_applies_to_both_views(self):
    payload = 'large "raw" reply \\ ' * 6000
    ecus = [{'tx_address': address, 'dtc_read': False, 'queries': [], 'codes': [], 'opaque': payload}
            for address in ('0x715', '0x7e0')]
    self.store.save('b' * 32, evidence(ecus=ecus))
    report = self.store.get_report(ecu='0x715')
    self.assertEqual([i['tx_address'] for i in report['items'] if i['kind'] == 'ecu'], ['0x715'])
    page = self.store.get_evidence(ecu='0x715', raw=True, limit=2)
    items, fragments = [], []
    while True:
      self.assertLessEqual(len(json.dumps(page).encode()), 8000)
      for item in page['items']:
        if item['kind'] == 'json_fragment':
          fragments.append(item['text'])
          if item['final']:
            items.append(json.loads(''.join(fragments)))
            fragments = []
        else:
          items.append(item)
      if page['next_cursor'] is None:
        break
      page = self.store.get_evidence('b' * 32, ecu='0x715', raw=True, limit=2, cursor=page['next_cursor'])
    self.assertTrue(any(i['value'] == payload for i in items))
    self.assertNotIn('0x7e0', json.dumps(items))
    self.assertFalse(fragments)
    with self.assertRaises(ValueError):
      self.store.get_evidence(raw=True, cursor='invalid')
    for limit in (0, 51, True):
      with self.assertRaises(ValueError):
        self.store.get_report(limit=limit)

  def test_latest_age_is_explicit_and_cursor_pins_saved_scan(self):
    self.store.save('c' * 32, evidence(started_at='2020-01-01T00:00:00+00:00', warnings=['one', 'two', 'three']))
    page = self.store.get_report(limit=1)
    self.assertGreater(page['report_age_seconds'], 86400)
    self.assertIsNone(page['active_scan_id'])
    self.assertFalse(page['is_active_scan'])
    self.assertEqual(page['restoration']['state'], 'unknown')
    self.store.save('d' * 32, evidence())
    following = self.store.get_report(cursor=page['next_cursor'])
    self.assertEqual(following['scan_id'], 'c' * 32)
    with self.assertRaises(ValueError):
      self.store.get_report('d' * 32, cursor=page['next_cursor'])

  def legacy_report(self, scan_id, version=None):
    # Real disk fixtures represent data produced before the currently running server.
    bundle = {'report': {'scan_id': scan_id, 'legacy_marker': True}, 'evidence': {}}
    if version is not None:
      bundle['server_version'] = version
    path = Path(self.tmp.name) / f'{scan_id}.json'
    path.write_text(json.dumps(bundle))
    return path

  def test_latest_skips_missing_different_and_invalid_versions_without_rewriting_files(self):
    self.store.save('a' * 32, evidence())
    os.utime(Path(self.tmp.name) / f'{"a" * 32}.json', (1, 1))
    files = [self.legacy_report(f'{i:032x}', version) for i, version in enumerate((None, '2.0.0', '99.0.0', 42), 1)]
    snapshots = {path: path.read_bytes() for path in files}
    for read in (self.store.get_report, self.store.get_evidence):
      page = read()
      self.assertEqual(page['scan_id'], 'a' * 32)
      self.assertEqual(page['server_version'], SERVER_VERSION)
    self.assertEqual(self.store.operation_status('latest')['scan_id'], 'a' * 32)
    self.assertEqual({path: path.read_bytes() for path in files}, snapshots)

  def test_incompatible_explicit_ids_and_empty_compatible_history_give_clear_errors(self):
    for i, version in enumerate((None, '2.0.0', '99.0.0', True)):
      scan_id = f'{i:032x}'
      self.legacy_report(scan_id, version)
      for read in (self.store.get, self.store.get_report, self.store.get_evidence, self.store.operation_status):
        with self.subTest(version=version, reader=read.__name__), self.assertRaisesRegex(ValueError, 'Incompatible report'):
          read(scan_id)
    for read in (self.store.get_report, self.store.get_evidence, self.store.operation_status):
      with self.assertRaisesRegex(ValueError, 'No compatible saved reports.*new scan'):
        read('latest')

  def test_cursor_cannot_bypass_version_check(self):
    self.store.save('a' * 32, evidence(warnings=['first', 'second']))
    cursor = self.store.get_report(limit=1)['next_cursor']
    self.legacy_report('a' * 32, '2.0.0')
    with self.assertRaisesRegex(ValueError, 'Incompatible report'):
      self.store.get_report(cursor=cursor)
