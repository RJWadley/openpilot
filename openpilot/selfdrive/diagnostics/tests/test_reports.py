import json
import tempfile
import unittest

from openpilot.selfdrive.diagnostics.manager import ReportStore
from openpilot.selfdrive.diagnostics.tests.test_manager import evidence


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
