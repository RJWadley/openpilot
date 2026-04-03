from cereal import car
from openpilot.common.realtime import DT_CTRL

ButtonType = car.CarState.ButtonEvent.Type

DISTANCE_BUTTON_DEBUG_ACCEL = 1.5
DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS = 1.0


class DistanceButtonDebug:
  def __init__(self):
    self.button_pressed = False
    self.start_from_stop = False
    self.standstill_timer = 0.0

  def update(self, CS):
    if CS.standstill:
      self.standstill_timer += DT_CTRL
    else:
      self.standstill_timer = 0.0

    for be in CS.buttonEvents:
      if be.type != ButtonType.gapAdjustCruise:
        continue

      self.button_pressed = be.pressed
      if be.pressed:
        self.start_from_stop = self.standstill_timer > DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS
      else:
        self.start_from_stop = False

  def get_long_override(self, a_target: float, should_stop: bool) -> tuple[float, bool, bool, int]:
    if not self.button_pressed:
      return a_target, should_stop, False, 0

    if self.start_from_stop:
      return max(a_target, DISTANCE_BUTTON_DEBUG_ACCEL), False, True, 1

    return min(a_target, -DISTANCE_BUTTON_DEBUG_ACCEL), True, False, -1
