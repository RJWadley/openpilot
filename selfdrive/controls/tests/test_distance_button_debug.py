from types import SimpleNamespace

from cereal import car

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.distance_button_debug import DistanceButtonDebug, DISTANCE_BUTTON_DEBUG_ACCEL, DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS

ButtonType = car.CarState.ButtonEvent.Type


def make_cs(*, standstill=False, button_events=None):
  return SimpleNamespace(standstill=standstill, buttonEvents=button_events or [])


def make_button_event(pressed):
  return SimpleNamespace(type=ButtonType.gapAdjustCruise, pressed=pressed)


def test_distance_button_forces_stop_while_moving():
  debug = DistanceButtonDebug()
  debug.update(make_cs(standstill=False, button_events=[make_button_event(True)]))

  a_target, should_stop, ignore_cruise_standstill, forced_accel, forced_max_planned_speed = debug.get_long_override(0.3, False)
  assert a_target == -DISTANCE_BUTTON_DEBUG_ACCEL
  assert should_stop
  assert not ignore_cruise_standstill
  assert forced_accel == -DISTANCE_BUTTON_DEBUG_ACCEL
  assert forced_max_planned_speed is None


def test_distance_button_forces_start_after_standstill():
  debug = DistanceButtonDebug()
  for _ in range(int(DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS / DT_CTRL) + 1):
    debug.update(make_cs(standstill=True))

  debug.update(make_cs(standstill=True, button_events=[make_button_event(True)]))

  a_target, should_stop, ignore_cruise_standstill, forced_accel, forced_max_planned_speed = debug.get_long_override(-0.3, True)
  assert a_target == DISTANCE_BUTTON_DEBUG_ACCEL
  assert not should_stop
  assert ignore_cruise_standstill
  assert forced_accel == DISTANCE_BUTTON_DEBUG_ACCEL
  assert forced_max_planned_speed == 10.0


def test_distance_button_override_clears_on_release():
  debug = DistanceButtonDebug()
  debug.update(make_cs(standstill=False, button_events=[make_button_event(True)]))
  debug.update(make_cs(standstill=False, button_events=[make_button_event(False)]))

  a_target, should_stop, ignore_cruise_standstill, forced_accel, forced_max_planned_speed = debug.get_long_override(0.2, False)
  assert a_target == 0.2
  assert not should_stop
  assert not ignore_cruise_standstill
  assert forced_accel is None
  assert forced_max_planned_speed is None
