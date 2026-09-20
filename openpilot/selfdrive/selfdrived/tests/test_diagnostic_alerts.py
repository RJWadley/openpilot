"""Exercise the real event creation and alert arbitration used by selfdrived."""
import unittest

from opendbc.car.structs import car
from openpilot.cereal import messaging
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import Events, EventName, ET


class TestDiagnosticAlerts(unittest.TestCase):
  def displayed(self, names, *, phase='scanning', preparing=False, steer=False, cruise=False, types=None):
    events, manager = Events(), AlertManager()
    cs = car.CarState.new_message(steerFaultPermanent=steer, accFaulted=cruise)
    message = messaging.new_message('diagnosticState')
    message.diagnosticState.phase = phase
    message.diagnosticState.preparingDiagnostics = preparing
    sm = {'diagnosticState': message.diagnosticState}
    for frame in range(200):
      events.clear()
      for name in names:
        events.add(name)
      manager.add_many(frame, events.create_alerts(types or [ET.PERMANENT], [None, cs, sm, True, 0, None]))
      manager.process_alerts(frame, set())
    return manager.current_alert

  def test_diagnostic_status_is_visible_with_lkas_fault_and_keeps_fault_context(self):
    alert = self.displayed([EventName.diagnosticsRunning, EventName.steerUnavailable], steer=True)
    self.assertIn('Diagnostics', alert.alert_text_1)
    self.assertIn('LKAS fault', alert.alert_text_2)

  def test_cruise_fault_remains_visible_in_diagnostic_banner(self):
    alert = self.displayed([EventName.diagnosticsRunning, EventName.accFaulted], cruise=True)
    self.assertIn('Diagnostics', alert.alert_text_1)
    self.assertIn('Cruise fault', alert.alert_text_2)

  def test_combined_faults_remain_visible_and_preparation_is_labeled(self):
    alert = self.displayed([EventName.diagnosticsRunning, EventName.steerUnavailable, EventName.accFaulted],
                           phase='preparing', steer=True, cruise=True)
    self.assertEqual(alert.alert_text_1, 'preparing diagnostics')
    self.assertIn('LKAS + Cruise faults reported', alert.alert_text_2)

  def test_discovery_display_does_not_change_native_scanning_or_hide_faults(self):
    alert = self.displayed([EventName.diagnosticsRunning, EventName.steerUnavailable], preparing=True, steer=True)
    self.assertEqual(alert.alert_text_1, 'preparing diagnostics')
    self.assertIn('LKAS fault', alert.alert_text_2)
    alert = self.displayed([EventName.diagnosticsRunning], phase='restoring', preparing=True)
    self.assertEqual(alert.alert_text_1, 'Diagnostics: Restoring')

  def test_restoring_and_stale_idle_never_claim_restoration_complete(self):
    for phase in ('restoring', 'idle'):
      alert = self.displayed([EventName.diagnosticsRunning], phase=phase)
      self.assertIn('blocked', alert.alert_text_2.lower())
      self.assertNotIn('restoration complete', alert.alert_text_2.lower())
    alert = self.displayed([EventName.diagnosticsRunning], phase='restoring')
    self.assertIn('Restoring', alert.alert_text_1)

  def test_immediate_safety_alert_still_wins(self):
    alert = self.displayed([EventName.diagnosticsRunning, EventName.steerUnavailable], steer=True,
                           types=[ET.PERMANENT, ET.IMMEDIATE_DISABLE])
    self.assertEqual(alert.alert_text_1, 'TAKE CONTROL IMMEDIATELY')

  def test_lkas_alert_returns_when_diagnostics_is_absent(self):
    alert = self.displayed([EventName.steerUnavailable], steer=True)
    self.assertEqual(alert.alert_text_1, 'LKAS Fault: Restart the car to engage')
