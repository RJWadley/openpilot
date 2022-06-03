from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.volkswagen.values import CAR, BUTTON_STATES, CANBUS, NetworkLocation, TransmissionType, GearShifter, PQ_CARS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    self.buttonStatesPrev = BUTTON_STATES.copy()

    # Alias Extended CAN parser to PT/CAM parser, based on detected network location
    self.cp_ext = self.cp if CP.networkLocation == NetworkLocation.fwdCamera else self.cp_cam

    if CP.networkLocation == NetworkLocation.fwdCamera:
      self.ext_bus = CANBUS.pt
      self.cp_ext = self.cp
    else:
      self.ext_bus = CANBUS.cam
      self.cp_ext = self.cp_cam

    self.pqCounter = 0
    self.wheelGrabbed = False
    self.pqBypassCounter = 0

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0


  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "volkswagen"
    ret.communityFeature = True  # technically unsupported, the best kind of unsupported

    ret.radarOffCan = True
    # Check for Comma Pedal
    ret.enableGasInterceptor = False

    if candidate in PQ_CARS:
      # Set global PQ35/PQ46/NMS parameters
      ret.safetyModel = car.CarParams.SafetyModel.volkswagenPq
      ret.enableBsm = 0x3BA in fingerprint[0]

      if 0x440 in fingerprint[0]:  # Getriebe_1 detected: traditional automatic or DSG gearbox
        ret.transmissionType = TransmissionType.automatic
      else:  # No trans message at all, must be a true stick-shift manual
        ret.transmissionType = TransmissionType.manual

      if 0x1A0 in fingerprint[1] or 0xAE in fingerprint[1]:
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

    else:
      # Set global MQB parameters
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volkswagen)]
      ret.enableBsm = 0x30F in fingerprint[0]  # SWA_01

      if 0xAD in fingerprint[0]:  # Getriebe_11 detected: traditional automatic or DSG gearbox
        ret.transmissionType = TransmissionType.automatic
      elif 0x187 in fingerprint[0]:  # EV_Gearshift detected: e-Golf or similar direct-drive electric
        ret.transmissionType = TransmissionType.direct
      else:
        ret.transmissionType = TransmissionType.manual

      if any(msg in fingerprint[1] for msg in [0x40, 0x86, 0xB2, 0xFD]):  # Airbag_01, LWI_01, ESP_19, ESP_21
        ret.networkLocation = NetworkLocation.gateway
      else:
        ret.networkLocation = NetworkLocation.fwdCamera

    # Global tuning defaults, can be overridden per-vehicle

    ret.steerActuatorDelay = 0.05
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    tire_stiffness_factor = 1.0  # Let the params learner figure this out
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kf = 0.00006
    ret.lateralTuning.pid.kpV = [0.6]
    ret.lateralTuning.pid.kiV = [0.2]

    # Per-chassis tuning values, override tuning defaults here if desired

    if candidate == CAR.ARTEON_MK1:
      ret.mass = 1733 + STD_CARGO_KG
      ret.wheelbase = 2.84

    elif candidate == CAR.ATLAS_MK1:
      ret.mass = 2011 + STD_CARGO_KG
      ret.wheelbase = 2.98

    elif candidate == CAR.GOLF_MK6:
      # Averages of all 1K/5K/AJ Golf variants
      ret.mass = 1379 + STD_CARGO_KG
      ret.wheelbase = 2.58
      ret.minSteerSpeed = 20 * CV.KPH_TO_MS  # May be lower depending on model-year/EPS FW
      ret.enableGasInterceptor = True

      # OP LONG parameters
      ret.openpilotLongitudinalControl = True
      ret.longitudinalTuning.deadzoneBP = [0.]
      ret.longitudinalTuning.deadzoneV = [0.]
      ret.longitudinalTuning.kpBP = [5., 35.]
      ret.longitudinalTuning.kpV = [2.8, 1.5]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.36]

      # PQ lateral tuning HCA_Status 7
      ret.lateralTuning.pid.kpBP = [0., 14., 20.]
      ret.lateralTuning.pid.kiBP = [0., 14., 20.]
      ret.lateralTuning.pid.kpV = [0.12, 0.135, 0.147]
      ret.lateralTuning.pid.kiV = [0.09, 0.10, 0.11]

    elif candidate == CAR.GOLF_MK7:
      ret.mass = 1397 + STD_CARGO_KG
      ret.wheelbase = 2.62

    elif candidate == CAR.JETTA_MK7:
      ret.mass = 1328 + STD_CARGO_KG
      ret.wheelbase = 2.71

    elif candidate == CAR.PASSAT_MK8:
      ret.mass = 1551 + STD_CARGO_KG
      ret.wheelbase = 2.79
    elif candidate == CAR.PASSAT_NMS:
      # Averages of all A3 Passat NMS
      ret.mass = 1503 + STD_CARGO_KG
      ret.wheelbase = 2.80
      ret.minSteerSpeed = 50 * CV.KPH_TO_MS  # May be lower depending on model-year/EPS FW
    elif candidate == CAR.TAOS_MK1:
      ret.mass = 1498 + STD_CARGO_KG
      ret.wheelbase = 2.69
    elif candidate == CAR.TCROSS_MK1:
      ret.mass = 1150 + STD_CARGO_KG
      ret.wheelbase = 2.60

    elif candidate == CAR.TIGUAN_MK2:
      ret.mass = 1715 + STD_CARGO_KG
      ret.wheelbase = 2.74

    elif candidate == CAR.TOURAN_MK2:
      ret.mass = 1516 + STD_CARGO_KG
      ret.wheelbase = 2.79

    elif candidate == CAR.TRANSPORTER_T61:
      ret.mass = 1926 + STD_CARGO_KG
      ret.wheelbase = 3.00  # SWB, LWB is 3.40, TBD how to detect difference
      ret.minSteerSpeed = 14.0

    elif candidate == CAR.AUDI_A3_MK3:
      ret.mass = 1335 + STD_CARGO_KG
      ret.wheelbase = 2.61

    elif candidate == CAR.AUDI_Q2_MK1:
      ret.mass = 1205 + STD_CARGO_KG
      ret.wheelbase = 2.61

    elif candidate == CAR.AUDI_Q3_MK2:
      # Averages of all 8U/F3/FS Q3 variants
      ret.mass = 1623 + STD_CARGO_KG
      ret.wheelbase = 2.68

    elif candidate == CAR.SEAT_ATECA_MK1:
      ret.mass = 1900 + STD_CARGO_KG
      ret.wheelbase = 2.64

    elif candidate == CAR.SEAT_LEON_MK3:
      ret.mass = 1227 + STD_CARGO_KG
      ret.wheelbase = 2.64

    elif candidate == CAR.SKODA_KAMIQ_MK1:
      ret.mass = 1265 + STD_CARGO_KG
      ret.wheelbase = 2.66

    elif candidate == CAR.SKODA_KAROQ_MK1:
      ret.mass = 1278 + STD_CARGO_KG
      ret.wheelbase = 2.66

    elif candidate == CAR.SKODA_KODIAQ_MK1:
      ret.mass = 1569 + STD_CARGO_KG
      ret.wheelbase = 2.79

    elif candidate == CAR.SKODA_OCTAVIA_MK3:
      ret.mass = 1388 + STD_CARGO_KG
      ret.wheelbase = 2.68

    elif candidate == CAR.SKODA_SCALA_MK1:
      ret.mass = 1192 + STD_CARGO_KG
      ret.wheelbase = 2.65

    elif candidate == CAR.SKODA_SUPERB_MK3:
      ret.mass = 1505 + STD_CARGO_KG
      ret.wheelbase = 2.84

    else:
      raise ValueError("unsupported car %s" % candidate)

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.centerToFront = ret.wheelbase * 0.45
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    buttonEvents = []

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_ext, self.CP.transmissionType)
    ret.canValid = True #self.cp.can_valid and self.cp_cam.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # TODO: add a field for this to carState, car interface code shouldn't write params
    # Update the device metric configuration to match the car at first startup,
    # or if there's been a change.
    #if self.CS.displayMetricUnits != self.displayMetricUnitsPrev:
    #  put_nonblocking("IsMetric", "1" if self.CS.displayMetricUnits else "0")

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic])

    # PQTIMEBOMB STUFF START
    # Warning alert for the 6min timebomb found on PQ's
    ret.stopSteering = False
    if True:  # (self.frame % 100) == 0: # Set this to false/False if you want to turn this feature OFF!
      if ret.cruiseState.enabled:
        self.pqCounter += 1
      else:
        self.pqCounter = 0
      if self.pqCounter >= 330 * 100:  # time in seconds until counter threshold for pqTimebombWarn alert
        if not self.wheelGrabbed:
          events.add(EventName.pqTimebombWarn)
          if self.pqCounter >= 345 * 100:  # time in seconds until pqTimebombTERMINAL
            events.add(EventName.pqTimebombTERMINAL)
            if self.pqCounter >= 359 * 100:  # time in seconds until auto bypass
              self.wheelGrabbed = True
        if self.wheelGrabbed or ret.steeringPressed:
          self.wheelGrabbed = True
          ret.stopSteering = True
          self.pqBypassCounter += 1
          if self.pqBypassCounter >= 1.05 * 100:  # time alloted for bypass
            self.wheelGrabbed = False
            self.pqCounter = 0
            self.pqBypassCounter = 0
            events.add(EventName.pqTimebombBypassed)
          else:
            events.add(EventName.pqTimebombBypassing)

    # PQTIMEBOMB STUFF END

    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.add(EventName.parkBrake)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    ret.buttonEvents = buttonEvents

    # update previous car states
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, self.ext_bus, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible,
                   c.hudControl.leftLaneDepart,
                   c.hudControl.rightLaneDepart)
    self.frame += 1
    return can_sends
