from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL

ButtonType = car.CarState.ButtonEvent.Type

DISTANCE_BUTTON_DEBUG_ACCEL = 0.3
DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS = 1.0


class DistanceButtonDebug:
  def __init__(self):
    self.forced_accel = 0.0
    self.standstill_timer = 0.0

  def update(self, CS):
    if CS.standstill:
      self.standstill_timer += DT_CTRL
    else:
      self.standstill_timer = 0.0

    for be in CS.buttonEvents:
      if be.type != ButtonType.gapAdjustCruise:
        continue

      if be.pressed:
        stopped_long_enough = self.standstill_timer > DISTANCE_BUTTON_DEBUG_STOPPED_SECONDS
        self.forced_accel = DISTANCE_BUTTON_DEBUG_ACCEL if stopped_long_enough else -DISTANCE_BUTTON_DEBUG_ACCEL
      else:
        self.forced_accel = 0.0

  def get_long_override(self, a_target: float, should_stop: bool) -> tuple[float, bool, bool, float | None]:
    if self.forced_accel == 0.0:
      return a_target, should_stop, False, None

    if self.forced_accel > 0.0:
      return max(a_target, self.forced_accel), False, True, self.forced_accel

    return min(a_target, self.forced_accel), True, False, self.forced_accel
