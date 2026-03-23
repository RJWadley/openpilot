import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class MQBStandstillManager:
  """
  Standstill handling for MQB ACC type 1. Ported from RJ's implementation, adapted for
  sunnypilot and cars without ESP_15 (no EPB, no hold torque signal).

  Hold request state machine (ACC_07 HMS signal):
    The ESP only engages hill hold in response to HMS=1 (hold request). Previous versions
    sent HMS=4 (start) or HMS=0 immediately at standstill, preventing ESP from ever
    engaging hold — confirmed cause of rollback on hills (zero hold frames in drive logs).

    State machine per standstill event:
      1. REQUESTING (no hold yet, within timeout):
         No override → raw stopping=True → HMS=1 (hold request)
         ESP engages hold within 2-5 frames on flat/hill.
      2. HOLDING (hold confirmed):
         esp_starting_override=False → HMS=3 (standby, never fights active hold)
      3. RELEASING (hold was confirmed, now dropped — normal ESP cycling):
         esp_starting_override=True → HMS=4 (release, harmless noop without hold)
      4. TIMEOUT (no hold after HOLD_REQUEST_TIMEOUT frames ~400ms):
         Fall back to HMS=4 permanently for this stop. Prevents SRBM accumulation
         on stops where ESP never engages hold (e.g. very gentle grade, slow creep).

    SRBM safety: HMS=1 without hold only accumulates SRBM after hundreds of sustained
    frames. The requesting window is ≤20 frames (400ms) before either hold confirms
    or we fall back to HMS=4. Zero SRBM risk confirmed from prior drive log analysis.

  Sunnypilot vs RJ differences:
    - Removed CS.esp_hold_uphill (ESP_15 — crashes this car variant)
    - Removed CS.esp_hold_torque_nm / CS.actual_torque_nm (ESP_15)
    - Replaced I-controller with fixed UPHILL_ACCEL_FLOOR
    - Simplified standstill detection: at_standstill = standstill OR vEgo < 0.5
    - Removed CS.distance_button_pressed (debug helper)
    - Signatures: __init__(CCP) unchanged; CarController uses (CP_SP) and (CC_SP)
  """

  HOLD_REQUEST_TIMEOUT = 20    # frames of HMS=1 before fallback to HMS=4 (~400ms at 50Hz)
  UPHILL_ACCEL_FLOOR = 1.5     # m/s² accel floor on uphill launch

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._hold_request_frames = 0
    self._hold_acquired = False  # latches True once hold confirms; prevents re-requesting
    self._hold_lost_frames = 0   # frames since hold last confirmed; resets _hold_acquired after timeout

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool
             ) -> tuple[bool, float, bool, bool, bool | None, bool | None]:
    esp_starting_override: bool | None = None
    esp_stopping_override: bool | None = None

    # CS.out.standstill = pcmCruise AND esp_hold_confirmation, always False on a hill
    # before hold is acquired. Use vEgo threshold so the override engages as soon as
    # the car is physically stopped regardless of ESP hold state.
    at_standstill = CS.out.standstill or CS.out.vEgo < 0.5

    # acc type 1 is sensitive to control signals when brake is pressed (preEnabled)
    if CS.out.brakePressed:
      long_active = False

    # Uphill launch: apply accel floor so engine torque builds before ESP releases hold
    if long_active and accel > 0 and self._prev_hold_confirmation and at_standstill:
      accel = max(accel, self.UPHILL_ACCEL_FLOOR)

    if long_active and at_standstill and stopping and not starting:
      if CS.esp_hold_confirmation:
        # State 2: HOLDING — standby, never fight active hold → HMS=3
        esp_starting_override = False
        esp_stopping_override = False
        self._hold_request_frames = 0
        self._hold_acquired = True   # latch: prevents re-requesting on rapid-cycle grades
        self._hold_lost_frames = 0
      elif self._prev_hold_confirmation:
        # State 3: RELEASING — hold just dropped, signal release → HMS=4
        esp_starting_override = True
        esp_stopping_override = False
        self._hold_request_frames = 0
      elif self._hold_acquired:
        # State 4a: POST-ACQUISITION — hold was acquired but not currently confirmed
        # Count frames since hold last confirmed. After 100 frames (~2s), reset
        # _hold_acquired so a fresh REQUESTING cycle can re-engage hold.
        # Exception: if the car is still rolling (vEgo > 0.3 m/s) we're on a grade
        # too steep for hold to engage — don't reset, stay on HMS=4 permanently
        # until departure. Resetting causes repeated REQUESTING→TIMEOUT oscillation
        # on steep declines (confirmed: 3-cycle loop at -14.4% grade).
        self._hold_lost_frames += 1
        if self._hold_lost_frames >= 100 and CS.out.vEgo < 0.3:
          self._hold_acquired = False
          self._hold_lost_frames = 0
          self._hold_request_frames = 0
          # fall into REQUESTING on next frame
        else:
          esp_starting_override = True
          esp_stopping_override = False
      elif self._hold_request_frames >= self.HOLD_REQUEST_TIMEOUT:
        # State 4b: TIMEOUT — hold never came, safe HMS=4 fallback
        esp_starting_override = True
        esp_stopping_override = False
      else:
        # State 1: REQUESTING — no override, allow HMS=1 to request hold from ESP
        self._hold_request_frames += 1
        # esp_starting_override remains None → raw stopping=True → HMS=1
    elif long_active and at_standstill and starting:
      # Departure: OP has transitioned to starting state — immediately signal HMS=4
      # regardless of hold state so ESP releases without waiting for hold to drop first
      esp_starting_override = True
      esp_stopping_override = False
      self._hold_request_frames = 0
    else:
      self._hold_request_frames = 0
      self._hold_acquired = False   # reset when leaving standstill
      self._hold_lost_frames = 0

    self._prev_hold_confirmation = CS.esp_hold_confirmation

    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)
    self.standstill_manager = MQBStandstillManager(self.CCP)
    self.distance_button_was_stopped = None

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
      hca_enabled = apply_torque != 0
      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        long_active = CC.longActive
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

        # distance button debug helper, force stop or start when distance button is pressed
        if self.CCS == mqbcan and CS.distance_button_pressed:
          if self.distance_button_was_stopped is None:
            self.distance_button_was_stopped = CS.out.standstill
          if long_active:
            if self.distance_button_was_stopped:
              accel = max(1.5, accel)
              stopping = False
              starting = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
            else:
              accel = min(-1.5, accel)
              stopping = CS.out.vEgo < self.CP.vEgoStopping if long_active else False
              starting = False
        else:
          self.distance_button_was_stopped = None

        esp_starting_override = None
        esp_stopping_override = None
        if self.CCS == mqbcan and CS.acc_type == 1:
          long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override = \
            self.standstill_manager.update(CS, long_active, accel, stopping, starting)

        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active)

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, long_active, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                           esp_starting_override, esp_stopping_override))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
