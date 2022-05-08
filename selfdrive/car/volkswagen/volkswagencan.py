import struct
from ctypes import create_string_buffer
from selfdrive.car import crc8_pedal
# ----------------------------------------------------------------------- #
#                                                                         #
# CAN message packing for MQB vehicles                                    #
#                                                                         #
# ----------------------------------------------------------------------- #

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_System_Status": 0x3,
    "LM_Offset": abs(apply_steer),
    "LM_Offset_Valid": lkas_enabled,
    "LM_Offsign": 1 if apply_steer < 0 else 0,
    "HCA_Control_Status": 0x5 if lkas_enabled else 0x3,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                           ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc):
  if hca_enabled:
    left_lane_hud = 3 if left_lane_visible else 1
    right_lane_hud = 3 if right_lane_visible else 1
  else:
    left_lane_hud = 2 if left_lane_visible else 1
    right_lane_hud = 2 if right_lane_visible else 1

  values = {
    "LDW_Status_LED_gelb": 1 if hca_enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if hca_enabled and not steering_pressed else 0,
    "LDW_SW_Info_links": left_lane_hud,
    "LDW_SW_Info_rechts": right_lane_hud,
    "LDW_Texte": hud_alert,
    "LDW_SW_Warnung_links": ldw_lane_warning_left,
    "LDW_SW_Warnung_rechts": ldw_lane_warning_right,
    "LDW_Seite_DLCTLC": ldw_side_dlc_tlc,
    "LDW_DLC": ldw_dlc,
    "LDW_TLC": ldw_tlc
  }
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_Typ468": CS.graTyp468,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }

  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)

# ----------------------------------------------------------------------- #
#                                                                         #
# CAN message packing for PQ35/PQ46/NMS vehicles                          #
#                                                                         #
# ----------------------------------------------------------------------- #

def create_pq_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "HCA_Zaehler": idx,
    "LM_Offset": abs(apply_steer),
    "LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_Status": 7 if (lkas_enabled and apply_steer != 0) else 3,
    "Vib_Freq": 16,
  }

  dat = packer.make_can_msg("HCA_1", bus, values)[2]
  values["HCA_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("HCA_1", bus, values)

def create_pq_braking_control(packer, bus, apply_brake, idx, brake_enabled, brake_pre_enable, stopping_wish):
  values = {
    "PQ_MOB_COUNTER": idx,
    "MOB_Bremsmom": abs(apply_brake),
    "MOB_Bremsstgr": abs(apply_brake),
    "MOB_Standby": 1 if (brake_enabled) else 0,
    "MOB_Freigabe": 1 if (brake_enabled and brake_pre_enable) else 0,
    "MOB_Anhaltewunsch": 1 if stopping_wish else 0,
  }

  dat = packer.make_can_msg("MOB_1", bus, values)[2]
  values["PQ_MOB_CHECKSUM"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4] ^ dat[5]
  return packer.make_can_msg("MOB_1", bus, values)

def create_pq_awv_control(packer, bus, idx, led_orange, led_green, abs_working):
  values = {
    "AWV_2_Fehler" : 1 if led_orange else 0,
    "AWV_2_Status" : 1 if led_green else 0,
    "AWV_Zaehler": idx,
    "AWV_Text": abs_working,
    "AWV_Infoton": 1 if (abs_working == 5) else 0,
    #"ACA_V_Wunsch": 30, #need to set this to v_ego if it works
    #"ACA_kmh_mph": 1, #sets ACC screen to MPH (i think)
    #"ACA_AnzDisplay": 1, #unsure of what this does, something about enabling a display maybe?
    #"ACA_Fahrerhinw": 0, #driver advice? perhaps asks user to take over
    #"ACA_gemZeitl": 0, #adding 1 more message, should display how far away a car is in seconds? not sure.
  }

  dat = packer.make_can_msg("mAWV", bus, values)[2]
  values["AWV_Checksumme"] = dat[1] ^ dat[2] ^ dat[3] ^ dat[4]
  return packer.make_can_msg("mAWV", bus, values)

def create_pq_pedal_control(packer, bus, apply_gas, idx):
  # Common gas pedal msg generator
  enable = apply_gas > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    apply_gas = apply_gas * 1125.
    if (apply_gas < 227):
      apply_gas = 227
    values["GAS_COMMAND"] = apply_gas
    values["GAS_COMMAND2"] = apply_gas

  dat = packer.make_can_msg("GAS_COMMAND", bus, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", bus, values)

def create_pq_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, leftLaneVisible, rightLaneVisible,
                           ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc):
  if hca_enabled:
    leftlanehud = 3 if leftLaneVisible else 1
    rightlanehud = 3 if rightLaneVisible else 1
  else:
    leftlanehud = 2 if leftLaneVisible else 1
    rightlanehud = 2 if rightLaneVisible else 1

  values = {
    "Right_Lane_Status": rightlanehud,
    "Left_Lane_Status": leftlanehud,
    "SET_ME_X1": 1,
    "Kombi_Lamp_Orange": 1 if hca_enabled and steering_pressed else 0,
    "Kombi_Lamp_Green": 1 if hca_enabled and not steering_pressed else 0,
    "LDW_Textbits": hud_alert,
  }
  return packer.make_can_msg("LDW_1", bus, values)

def create_pq_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Neu_Zaehler": idx,
    "GRA_Sender": CS.graSenderCoding,
    "GRA_Abbrechen": 1 if (buttonStatesToSend["cancel"] or CS.buttonStates["cancel"]) else 0,
    "GRA_Hauptschalt": CS.graHauptschalter,
  }

  dat = packer.make_can_msg("GRA_Neu", bus, values)[2]
  values["GRA_Checksum"] = dat[1] ^ dat[2] ^ dat[3]
  return packer.make_can_msg("GRA_Neu", bus, values)

def create_radar_VIN_msg(id,radarVIN,radarCAN,radarTriggerMessage,useRadar,radarPosition,radarEpasType):
  msg_id = 0x560
  msg_len = 8
  msg = create_string_buffer(msg_len)
  if id == 0:
    struct.pack_into('BBBBBBBB', msg, 0, id,radarCAN,useRadar + (radarPosition << 1) + (radarEpasType << 3), ((radarTriggerMessage >> 8) & 0xFF),(radarTriggerMessage & 0xFF),ord(radarVIN[0]),ord(radarVIN[1]),ord(radarVIN[2]))
  if id == 1:
    struct.pack_into('BBBBBBBB', msg, 0, id,ord(radarVIN[3]),ord(radarVIN[4]),ord(radarVIN[5]),ord(radarVIN[6]),ord(radarVIN[7]),ord(radarVIN[8]),ord(radarVIN[9]))
  if id == 2:
    struct.pack_into('BBBBBBBB', msg, 0, id,ord(radarVIN[10]),ord(radarVIN[11]),ord(radarVIN[12]),ord(radarVIN[13]),ord(radarVIN[14]),ord(radarVIN[15]),ord(radarVIN[16]))
  return [msg_id, 2, msg.raw, 0]

