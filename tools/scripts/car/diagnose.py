#!/usr/bin/env python3
"""Read OBD-II/UDS faults on a parked vehicle with openpilot stopped. See diagnose.md."""
import argparse
import gzip
import hashlib
import json
import math
import re
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, UTC
from functools import partial
from pathlib import Path


DATASET = Path(__file__).with_name("data") / "obdex.json.gz"
SCHEMA_VERSION = 4
STATUS_BITS = (
  "test_failed", "test_failed_this_operation_cycle", "pending", "confirmed",
  "test_not_completed_since_last_clear", "test_failed_since_last_clear",
  "test_not_completed_this_operation_cycle", "warning_indicator_requested",
)
STATUS_MEANINGS = {
  "test_failed": "The module's latest test result was a failure, not a continuous live measurement.",
  "test_failed_this_operation_cycle": "A test failed during the module's current operation cycle; it may have passed since.",
  "pending": "Pending: a failure was detected; this flag alone does not establish a confirmed or ongoing fault.",
  "confirmed": "Confirmed: the module's confirmation criteria were met; this alone does not mean the fault is happening now.",
  "test_not_completed_since_last_clear": "The test has not completed since codes were last cleared.",
  "test_failed_since_last_clear": "A failure occurred since codes were last cleared; this may be historical.",
  "test_not_completed_this_operation_cycle": "The test has not completed during the module's current operation cycle.",
  "warning_indicator_requested": "The module requests a warning indicator for this fault.",
  "stored": "Stored: a recorded fault, not proof that it is happening now.",
  "permanent": "Permanent emissions record: retained until the vehicle verifies resolution; the problem may already be repaired.",
}
DTC_QUERIES = ("uds_codes", "obd_stored", "obd_pending", "obd_permanent")
# Failure/pending/confirmed/history/lamp evidence. Bits 4 and 6 only say a test has not completed.
FAULT_STATUS_MASK = 0xAF
OBD_MODES = {0x03: "stored", 0x07: "pending", 0x0A: "permanent"}
IDENTIFIERS = {0xF187: "part_number", 0xF189: "software_version", 0xF197: "component"}
# Only frame zero is standardized for this initial freeze-frame reader.
FREEZE_PIDS = {0x04: ("load", "%"), 0x05: ("coolant_temperature", "C"), 0x0C: ("engine_speed", "rpm"),
               0x0D: ("vehicle_speed", "km/h"), 0x0F: ("intake_air_temperature", "C")}
FIXED_REQUESTS = {b"\x03", b"\x07", b"\x0a", b"\x01\x00", b"\x01\x01", b"\x3e\x00",
                  b"\x10\x01", b"\x10\x03", b"\x19\x01\xff", b"\x19\x02\xff",
                  b"\x02\x00\x00", b"\x02\x02\x00"}
FIXED_REQUESTS |= {b"\x02" + bytes([pid, 0]) for pid in FREEZE_PIDS}
FIXED_REQUESTS |= {b"\x22" + did.to_bytes(2, "big") for did in IDENTIFIERS}
VIN_REQUESTS = {b"\x22\xf1\x90": b"\x62\xf1\x90", b"\x09\x02": b"\x49\x02\x01"}
FIXED_REQUESTS |= VIN_REQUESTS.keys()


@dataclass(frozen=True)
class Target:
  tx: int
  rx: int
  bus: int = 1
  obd: bool = True
  subaddress: int | None = None

  def as_dict(self):
    return {"tx_address": hex(self.tx), "rx_address": hex(self.rx), "bus": self.bus,
            "obd_multiplexing": self.obd, "subaddress": self.subaddress}


def target_key(target):
  return target.bus, target.obd, target.tx, target.rx, -1 if target.subaddress is None else target.subaddress


def rx_address(tx, offset=8):
  return tx + offset if tx <= 0x7FF else (tx & 0x1FFF0000) | ((tx & 0xFF) << 8) | ((tx >> 8) & 0xFF)


def valid_tx(address):
  # Same physical-address limits as Panda's ELM327 safety mode. No functional UDS requests.
  return address != 0x7DF and (0x600 <= address <= 0x7FF or address == 0x24B or
                             (0 <= address <= 0x1FFFFFFF and address & 0x1FFF00FF == 0x18DA00F1))


def read_request(request):
  return request in FIXED_REQUESTS or (len(request) == 6 and request[:2] in (b"\x19\x04", b"\x19\x06") and request[-1] == 0xFF)


def load_dataset():
  with gzip.open(DATASET, "rt", encoding="utf-8") as stream:
    return json.load(stream)


def format_obd_code(raw):
  if len(raw) != 2:
    raise ValueError("An OBD code must contain two bytes")
  return f"{'PCBU'[raw[0] >> 6]}{raw[0] & 0x3F:02X}{raw[1]:02X}"


def parse_obd_codes(data, status):
  # ISO 15765-4: the response body starts with a code count (unlike legacy OBD protocols).
  if not data or len(data) < 1 + data[0] * 2:
    raise ValueError("Truncated OBD DTC count/records")
  end = 1 + data[0] * 2
  if any(data[end:]):
    raise ValueError("Nonzero data beyond the OBD DTC count")
  result = []
  for i in range(1, end, 2):
    raw = data[i:i + 2]
    if raw == b"\x00\x00":
      raise ValueError("Zero DTC inside the declared OBD code count")
    result.append({"protocol": "obd", "raw_dtc": raw.hex(), "code": format_obd_code(raw), "status": [status]})
  return result


def parse_uds_count(data):
  if len(data) != 4:
    raise ValueError("Invalid UDS DTC-count response length")
  return {"status_availability": data[0], "format": data[1], "count": int.from_bytes(data[2:], "big")}


def parse_uds_codes(data, dtc_format):
  if not data or (len(data) - 1) % 4:
    raise ValueError("Truncated UDS DTC/status records")
  result = []
  for i in range(1, len(data), 4):
    raw, status = data[i:i + 3], data[i + 3]
    if not status & data[0] & FAULT_STATUS_MASK:
      continue
    record = {"protocol": "uds", "raw_dtc": raw.hex(), "code": f"0x{raw.hex().upper()}",
              "format": dtc_format, "status_byte": status, "status_availability": data[0],
              "status": [name for bit, name in enumerate(STATUS_BITS) if status & data[0] & (1 << bit)]}
    # Do not assume all three-byte UDS codes use SAE's P/C/B/U representation.
    if dtc_format in (0, 4):
      record.update(code=format_obd_code(raw[:2]), failure_type=f"0x{raw[2]:02X}")
    result.append(record)
  return result


def parse_monitor_status(data):
  if len(data) != 4:
    raise ValueError("Invalid OBD monitor-status length")
  return {"mil_on": bool(data[0] & 0x80), "stored_dtc_count": data[0] & 0x7F, "monitor_status_raw": data[1:].hex()}


def parse_freeze_value(pid, data):
  if len(data) != (2 if pid == 0x0C else 1):
    raise ValueError("Invalid freeze-frame value length")
  if pid == 0x0C:
    value = int.from_bytes(data, "big") / 4
  elif pid == 0x04:
    value = data[0] * 100 / 255
  elif pid in (0x05, 0x0F):
    value = data[0] - 40
  else:
    value = data[0]
  name, unit = FREEZE_PIDS[pid]
  return {"name": name, "value": value, "unit": unit, "frame": 0}


def parse_uds_detail(data):
  if not data:
    raise ValueError("Missing DTC status in detail response")
  return {"status_byte": data[0], "records_raw": data[1:].hex()}


class DiagnosticError(Exception):
  def __init__(self, outcome, message, nrc=None):
    super().__init__(message)
    self.outcome = outcome
    self.nrc = nrc


class PandaTransport:
  """Reuse opendbc's ISO-TP while imposing an absolute deadline, including response-pending."""
  def __init__(self, panda, timeout, deadline):
    from opendbc.car.uds import CanClient, IsoTpMessage, InvalidSubAddressError
    self.panda = panda
    self.timeout = timeout
    self.deadline = deadline
    self.responses = []
    self.can_client_type = CanClient
    self.message_type = IsoTpMessage
    self.subaddress_error = InvalidSubAddressError

  def exchange(self, target, request):
    self.responses = []
    if not read_request(request):
      raise ValueError("Request is outside the diagnostic reader allowlist")
    if time.monotonic() >= self.deadline:
      raise DiagnosticError("not_queried", "Scan deadline reached")
    client = self.can_client_type(partial(self.panda.can_send, timeout=100), self.panda.can_recv,
                                  target.tx, target.rx, target.bus, target.subaddress)
    message = self.message_type(client)
    try:
      message.send(request)
      deadline = min(self.deadline, time.monotonic() + self.timeout)
      while time.monotonic() < deadline:
        response, _ = message.recv(timeout=0)
        if response is not None:
          self.responses.append(response.hex())
          # Delayed replies from an earlier discovery request must not complete
          # a different service's request. Keep them in the raw evidence.
          if response and ((response[0] == 0x7F and len(response) >= 2 and response[1] != request[0]) or
                           (response[0] != 0x7F and response[0] != request[0] + 0x40)):
            continue
          if response == bytes([0x7F, request[0], 0x78]):
            continue  # Never extend the deadline indefinitely for response-pending.
          return response
        time.sleep(0.001)
    except (AssertionError, self.subaddress_error) as e:
      raise DiagnosticError("malformed", str(e)) from e
    raise DiagnosticError("timeout", "No complete response before the request deadline")


class Reader:
  def __init__(self, transport, target):
    self.transport = transport
    self.target = target
    self.queries = []

  def read(self, name, request, prefix, decode=bytes.hex):
    query = {"name": name, "request": request.hex()}
    self.queries.append(query)
    try:
      response = self.transport.exchange(self.target, request)
      if response[:1] == b"\x7f":
        if len(response) != 3 or response[1] != request[0]:
          raise ValueError("Malformed or mismatched negative response")
        nrc = response[2]
        outcome = "unsupported" if nrc in (0x11, 0x12, 0x31) else "rejected"
        raise DiagnosticError(outcome, f"ECU rejected request (NRC 0x{nrc:02X})", nrc)
      if not response.startswith(prefix):
        raise ValueError(f"Expected response prefix {prefix.hex()}")
      query.update(outcome="ok", data=decode(response[len(prefix):]))
    except DiagnosticError as e:
      query.update(outcome=e.outcome, error=str(e))
      if e.nrc is not None:
        query["negative_response_code"] = e.nrc
    except (ValueError, IndexError) as e:
      query.update(outcome="malformed", error=str(e))
    except KeyboardInterrupt:
      query.update(outcome="interrupted", error="Query interrupted")
      raise
    except Exception as e:
      query.update(outcome="transport_error", error=f"{type(e).__name__}: {e}")
      raise
    finally:
      query["responses"] = list(self.transport.responses)
    return query


def enrich_codes(result, entries):
  """Keep third-party reference data separate from ECU evidence; never guess a DTC format."""
  identity = result.get("identity", {})
  context = " ".join(identity[name].strip() for name in ("component", "part_number") if identity.get(name))
  for record in result["codes"]:
    code = record["code"]
    failure = record.get("failure_type", "")[2:]
    record["display_code"] = f"{code}-{failure}" if failure else code
    entry = entries.get(code)
    if entry is not None:
      record["lookup"] = {"source": "OBDex", "title": entry["title"]["en"], "entry": entry}
      continue

    representations = [code]
    if failure:
      representations = [f"{code} {failure}", f"{code}{failure}", code]
    if record["protocol"] == "uds":
      # VCDS scan logs also print the full raw integer in decimal. This conversion
      # is lossless for every brand, unlike assuming its first two bytes are SAE.
      decimal, hexadecimal = str(int(record["raw_dtc"], 16)), f"0x{record['raw_dtc'].upper()}"
      if code.startswith("0x"):
        representations = [decimal, hexadecimal]
        record["display_code"] = f"{decimal} ({hexadecimal})"
      else:
        representations += [decimal, hexadecimal]
    record["search"] = {"codes": representations, "query": f"{representations[0]} {context}".strip()}


def query_ecu(transport, target, entries, details=False, max_details=16, obd=False):
  reader = Reader(transport, target)
  result = {**target.as_dict(), "codes": [], "queries": reader.queries, "dtc_read": False, "ignored_non_fault_records": 0}
  extended = False
  try:
    query = reader.read("uds_codes", b"\x19\x02\xff", b"\x59\x02", lambda data: parse_uds_codes(data, None))
    # A session change is needed on some ECUs, but not on every module. Never unlock security access.
    if query.get("negative_response_code") in (0x22, 0x7E, 0x7F):
      session = reader.read("extended_session", b"\x10\x03", b"\x50\x03")
      extended = session["outcome"] == "ok"
      if extended:
        query = reader.read("uds_codes", b"\x19\x02\xff", b"\x59\x02", lambda data: parse_uds_codes(data, None))
    if query["outcome"] == "ok":
      result["dtc_read"] = True
      count = reader.read("uds_count", b"\x19\x01\xff", b"\x59\x01", parse_uds_count)
      dtc_format = count["data"]["format"] if count["outcome"] == "ok" else None
      records = parse_uds_codes(bytes.fromhex(query["responses"][-1])[2:], dtc_format)
      result["ignored_non_fault_records"] = (len(bytes.fromhex(query["responses"][-1])) - 3) // 4 - len(records)
      query["data"] = records
      result["codes"].extend(dict(record) for record in records)
      result["uds_status_availability"] = bytes.fromhex(query["responses"][-1])[2]

    if obd and target.subaddress is None:
      for mode, status in OBD_MODES.items():
        query = reader.read(f"obd_{status}", bytes([mode]), bytes([mode + 0x40]), partial(parse_obd_codes, status=status))
        if query["outcome"] == "ok":
          result["dtc_read"] = True
          result["codes"].extend(dict(record) for record in query["data"])
      reader.read("obd_monitor_status", b"\x01\x01", b"\x41\x01", parse_monitor_status)

    # Every fault benefits from ECU-reported identity, not cross-brand address hints.
    if result["dtc_read"] and (details or result["codes"]):
      result["identity"] = {}
      for did, name in IDENTIFIERS.items():
        request = b"\x22" + did.to_bytes(2, "big")
        query = reader.read(name, request, b"\x62" + request[1:], lambda data: data.decode("utf-8", errors="replace").rstrip("\x00").strip())
        if query["outcome"] == "ok":
          result["identity"][name] = query["data"]

    # Collect the small decoded context before spending the budget on raw UDS details.
    if obd and (details or any(code["protocol"] == "obd" for code in result["codes"])):
      read_freeze_frame(reader)

    if details and result["dtc_read"]:
      uds_records = [code for code in result["codes"] if code["protocol"] == "uds"]
      result["details_omitted"] = max(0, len(uds_records) - max_details)
      for code in uds_records[:max_details]:
        raw = bytes.fromhex(code["raw_dtc"])
        for subfunction, name in ((4, "snapshot"), (6, "extended_data")):
          # Keep the ECU-specific record body raw; the response prefix validates the echoed DTC.
          reader.read(f"{name}_{code['raw_dtc']}", bytes([0x19, subfunction]) + raw + b"\xff",
                      bytes([0x59, subfunction]) + raw, parse_uds_detail)
  except KeyboardInterrupt:
    result["interrupted"] = True
  except Exception as e:
    result["error"] = f"{type(e).__name__}: {e}"
  finally:
    if extended:
      try:
        reader.read("default_session", b"\x10\x01", b"\x50\x01")
      except KeyboardInterrupt:
        result["interrupted"] = True
      except Exception as e:
        result.setdefault("error", f"Session cleanup failed: {e}")
  enrich_codes(result, entries)
  return result


def read_freeze_frame(reader):
  def bitmap(data):
    if len(data) != 4:
      raise ValueError("Invalid freeze-frame supported-PID bitmap")
    return int.from_bytes(data, "big")

  supported = reader.read("freeze_supported", b"\x02\x00\x00", b"\x42\x00\x00", bitmap)
  if supported["outcome"] != "ok":
    return
  # A frame is associated with this DTC, not with every code currently returned by the ECU.
  reader.read("freeze_dtc", b"\x02\x02\x00", b"\x42\x02\x00", format_obd_code)
  for pid in FREEZE_PIDS:
    if supported["data"] & (1 << (32 - pid)):
      reader.read(f"freeze_{FREEZE_PIDS[pid][0]}", bytes([2, pid, 0]), bytes([0x42, pid, 0]), partial(parse_freeze_value, pid))


def load_known_targets():
  """Addresses/routing only: never execute firmware-query sequences from a brand profile."""
  from opendbc.car.interfaces import get_interface_attr
  from opendbc.car.fw_query_definitions import ECU_NAME
  configs = get_interface_attr("FW_QUERY_CONFIG", ignore_none=True)
  versions = get_interface_attr("FW_VERSIONS", ignore_none=True)
  targets = defaultdict(set)
  for brand, config in configs.items():
    for ecu, address, subaddress in config.get_all_ecus(versions.get(brand, {})):
      if not valid_tx(address):
        continue
      for request in config.requests:
        if request.bus not in (0, 1, 2) or (request.whitelist_ecus and ecu not in request.whitelist_ecus):
          continue
        target = Target(address, rx_address(address, request.rx_offset), request.bus,
                        request.obd_multiplexing if request.bus == 1 else False, subaddress)
        targets[target].add(f"{brand}:{ECU_NAME.get(ecu, 'unknown')}")
  return {target: sorted(names) for target, names in targets.items()}


def generic_probes():
  # Physical request ranges permitted by Panda ELM327; no predicted reply IDs.
  addresses = [0x24B] + [address for address in range(0x600, 0x800) if address != 0x7DF]
  addresses += [0x18DA00F1 | (node << 8) for node in range(256) if node != 0xF1]
  return {(address, None) for address in addresses}


def selected_probes(args, known_targets, bus, obd):
  probes = {(t.tx, t.subaddress) for t in known_targets if (t.bus, t.obd) == (bus, obd)}
  if args.addr is None:
    return generic_probes() | probes
  return {(args.addr, args.subaddress)} | {(tx, sub) for tx, sub in probes if tx == args.addr and
                                        (args.subaddress is None or sub == args.subaddress)}


def single_frame(payload, subaddress=None):
  prefix = b"" if subaddress is None else bytes([subaddress])
  return (prefix + bytes([len(payload)]) + payload).ljust(8, b"\x00")


def frame_payload(data, subaddress=None):
  if subaddress is not None:
    if not data or data[0] != subaddress:
      return None
    data = data[1:]
  if not data or not 1 <= data[0] <= 7 or len(data) < data[0] + 1:
    return None
  return data[1:data[0] + 1]


def discover(panda, probes, bus, obd, deadline, probe_wait=0.1, timeout=1.0):
  """Learn physical TX/RX pairs, then corroborate them with a different read service.

  Tester-present replies don't echo the request address. Serialize probes, discard
  queued traffic between requests, and retain failed/ambiguous associations as
  evidence instead of assigning every reply to a guessed offset.
  """
  confirmed, emissions, sent, answered = set(), set(), set(), set()
  evidence, unconfirmed = [], []
  wire = PandaTransport(panda, timeout, deadline)

  def receive_window():
    until = min(deadline, time.monotonic() + probe_wait)
    while True:
      yield from ((address, data) for address, data, rx_bus in panda.can_recv() if rx_bus == bus and 0 <= address <= 0x1FFFFFFF)
      if time.monotonic() >= until:
        break
      time.sleep(min(0.002, max(0, until - time.monotonic())))

  if probes and time.monotonic() < deadline:
    panda.can_recv()  # Ignore responses queued before our requests.
    for address in (0x7DF, 0x18DB33F1):
      if time.monotonic() >= deadline:
        break
      panda.can_send(address, single_frame(b"\x01\x00"), bus, timeout=100)
    for address, data in receive_window():
      payload = frame_payload(data)
      if payload is None or len(payload) != 6 or payload[:2] != b"\x41\x00":
        continue
      if 0x7E8 <= address <= 0x7EF:
        tx = address - 8
      elif address & 0x1FFFFF00 == 0x18DAF100:
        tx = 0x18DA00F1 | ((address & 0xFF) << 8)
      else:
        continue
      # Only emissions OBD defines these mappings. Also respect --addr targeting.
      if (tx, None) in probes:
        target = Target(tx, address, bus, obd)
        emissions.add(target)
        evidence.append({**target.as_dict(), "request_address": hex(0x7DF if address <= 0x7FF else 0x18DB33F1),
                         "request": "0100", "response": payload.hex(), "outcome": "observed"})

  # Common 0x7xx addresses first so a short budget still covers that range.
  ordered = sorted(probes, key=lambda p: (not 0x700 <= p[0] <= 0x7FF, p[0], -1 if p[1] is None else p[1]))
  for tx, subaddress in ordered:
    if time.monotonic() >= deadline:
      break
    panda.can_recv()
    panda.can_send(tx, single_frame(b"\x3e\x00", subaddress), bus, timeout=100)
    sent.add((tx, subaddress))
    candidates = {t for t in emissions if (t.tx, t.subaddress) == (tx, subaddress)}
    for address, data in receive_window():
      payload = frame_payload(data, subaddress)
      if payload == b"\x7e\x00" or (payload is not None and len(payload) == 3 and payload[:2] == b"\x7f\x3e"):
        target = Target(tx, address, bus, obd, subaddress)
        if target not in candidates:
          evidence.append({**target.as_dict(), "request": "3e00", "response": payload.hex(), "outcome": "observed"})
        candidates.add(target)
    if candidates:
      answered.add((tx, subaddress))
    for target in sorted(candidates, key=target_key):
      reader = Reader(wire, target)
      requests = [("obd_confirmation", b"\x01\x00", b"\x41\x00")] if target in emissions else [
        ("identity_confirmation", b"\x22\xf1\x97", b"\x62\xf1\x97"),
        ("dtc_confirmation", b"\x19\x02\xff", b"\x59\x02"),
      ]
      for name, request, prefix in requests:
        if time.monotonic() >= deadline:
          break
        panda.can_recv()
        query = reader.read(name, request, prefix)
        evidence.append({**target.as_dict(), **query})
        # A correctly correlated rejection also demonstrates a diagnostic endpoint.
        if query["outcome"] in ("ok", "unsupported", "rejected"):
          confirmed.add(target)
          break
      if target not in confirmed:
        unconfirmed.append({**target.as_dict(), "reason": "No matching response to a different read service; possibly delayed traffic."})
    if len(sent) % 128 == 0:
      print(f"Discovery: {len(sent)}/{len(probes)} addresses probed, {len(confirmed)} reply pairs confirmed…", file=sys.stderr)

  by_request, by_reply = defaultdict(set), defaultdict(set)
  for target in confirmed:
    by_request[target.tx, target.subaddress].add(target)
    by_reply[target.rx, target.subaddress].add(target)
  ambiguous = {t for t in confirmed if len(by_request[t.tx, t.subaddress]) > 1 or len(by_reply[t.rx, t.subaddress]) > 1}
  found = confirmed - ambiguous
  unanswered = sent - answered
  return found, emissions & found, {"bus": bus, "obd_multiplexing": obd, "method": "sequential_learned",
                                   "probe_count": len(sent), "responses": evidence,
                                   "unanswered": [{"tx_address": hex(tx), "subaddress": sub} for tx, sub in ordered
                                                  if (tx, sub) in unanswered],
                                   "unconfirmed": unconfirmed,
                                   "ambiguous": [t.as_dict() for t in sorted(ambiguous, key=target_key)],
                                   "not_probed": len(probes - sent)}


def hardware_preflight(panda):
  health = panda.health()  # Also checks that the firmware health-packet version matches.
  internal = panda.is_internal()
  harness = health.get("car_harness_status")
  ignition_line, ignition_can = health.get("ignition_line"), health.get("ignition_can")
  checks = []
  if ignition_line or ignition_can:
    checks.append({"name": "ignition", "status": "pass", "message": "Ignition detected."})
  elif internal:
    checks.append({"name": "ignition", "status": "fail",
                   "message": "Ignition not detected. Turn the ignition fully on (not just accessory mode), then retry."})
  else:
    checks.append({"name": "ignition", "status": "warning",
                   "message": "External Panda has no active ignition signal. Verify ignition is on before scanning."})
  if internal:
    checks.append({"name": "harness", "status": "pass" if harness in (1, 2) else "fail",
                   "message": "Car harness detected." if harness in (1, 2) else
                   "Car harness not detected. Check the harness box and the OBD-C cable connecting it to the comma."})
  # The reported bitmask is telemetry, not the ELM327 transmit gate. In particular,
  # CAN interrupt-rate faults stay latched after the interrupt rate recovers.
  # Keep Panda's safety checks and the route-level connectivity checks in charge.
  faults = health["faults"]
  checks.append({"name": "panda_health", "status": "pass" if faults == 0 else "warning",
                 "message": "Panda health OK." if faults == 0 else
                 f"Panda reports fault flags (faults={faults}, {faults:#x}). Continuing read-only diagnostics; flags may be latched. " +
                 "Actual connection failures and Panda safety restrictions still apply."})
  return {"ready": not any(check["status"] == "fail" for check in checks), "checks": checks,
          "evidence": {"internal_panda": internal, "ignition_line": ignition_line, "ignition_can": ignition_can,
                       "car_harness_status": harness, "input_voltage_mv": health.get("voltage"), "faults": faults,
                       "fault_status": health.get("fault_status")}}


def check_obd_link(panda, deadline, wait=0.3):
  """Check the selected bus-1 OBD path, not the physical identity of a comma power adapter."""
  check = {"name": "obd_can", "status": "warning", "comma_power_present": None, "received_frames": 0,
           "message": "OBD CAN connectivity is unverified. Check comma power's OBD plug and RJ45 cable to the harness box. " +
                      "A silent bus can also mean sleeping ECUs, gateway restrictions, or unsupported diagnostics."}
  if time.monotonic() >= deadline:
    check["message"] = "OBD connectivity check not performed: scan deadline reached."
    return check
  try:
    before = panda.can_health(1)
  except (AttributeError, RuntimeError) as e:
    check["message"] += f" CAN health unavailable: {e}"
    return check
  check["can_health_before"] = before
  # Bus 1 is unchanged by harness orientation. TX counters and returned/echo frames
  # only mean queued for transmission in Panda firmware; they do NOT prove a CAN ACK.
  for address in (0x7DF, 0x18DB33F1):
    if time.monotonic() >= deadline:
      break
    panda.can_send(address, single_frame(b"\x01\x00"), 1, timeout=100)
  until = min(deadline, time.monotonic() + wait)
  while True:
    check["received_frames"] += sum(bus == 1 and bool(data) for _, data, bus in panda.can_recv())
    if time.monotonic() >= until:
      break
    time.sleep(0.002)
  after = panda.can_health(1)
  check["can_health_after"] = after
  fresh_errors = after["total_error_cnt"] > before["total_error_cnt"]
  ack_error = "AckError" in (after["last_error"], after["last_stored_error"])
  if check["received_frames"]:
    check.update(status="pass", message="OBD CAN traffic received. This confirms CAN activity, not access to every ECU or comma power identity.")
  elif after["bus_off"] or (fresh_errors and ack_error):
    check.update(status="fail", message="OBD CAN unavailable: no received traffic and the controller reports " +
                 "bus-off or new acknowledgement errors. Check comma power's OBD plug and RJ45 cable to the harness box " +
                 "(or equivalent OBD wiring), ignition, and CAN bitrate. Skipping this OBD route; harness routes can still be scanned.")
  return check


def selected_routes(args):
  buses = args.bus if args.bus is not None else ([1, 0, 2] if args.broad else [1])
  obd = args.obd or ("auto" if args.broad else "on")
  return list(dict.fromkeys((bus, mux) for bus in buses for mux in
                           ((True, False) if bus == 1 and obd == "auto" else (bus == 1 and obd != "off",))))


def module_cache_path():
  root = Path("/data") if Path("/AGNOS").is_file() else Path.home() / ".cache" / "openpilot"
  return root / "diagnostics" / "modules.json"


def cached_target(data):
  target = Target(int(data["tx_address"], 16), int(data["rx_address"], 16), data["bus"], data["obd_multiplexing"], data["subaddress"])
  if (not valid_tx(target.tx) or not 0 <= target.rx <= 0x1FFFFFFF or type(target.bus) is not int or target.bus not in (0, 1, 2) or
      type(target.obd) is not bool or (target.obd and target.bus != 1) or
      (target.subaddress is not None and (type(target.subaddress) is not int or not 0 <= target.subaddress <= 255))):
    raise ValueError("Invalid cached diagnostic address")
  return target


def load_module_cache(path):
  data = json.loads(path.read_text())
  if data["version"] != 1 or not isinstance(data["routes"], list):
    raise ValueError("Unsupported module cache format")
  identity = data["identity"]
  identity_target = cached_target(identity["target"])
  if bytes.fromhex(identity["request"]) not in VIN_REQUESTS or re.fullmatch(r"[0-9a-f]{64}", identity["vin_hash"]) is None:
    raise ValueError("Invalid cached vehicle identity")
  routes, all_targets = set(), set()
  for route in data["routes"]:
    bus, obd = route["bus"], route["obd_multiplexing"]
    if (type(bus) is not int or bus not in (0, 1, 2) or type(obd) is not bool or (obd and bus != 1) or (bus, obd) in routes or
        not isinstance(route["targets"], list) or not isinstance(route["scanned_at"], str)):
      raise ValueError("Invalid cached route")
    routes.add((bus, obd))
    requests, replies = set(), set()
    for item in route["targets"]:
      target = cached_target(item)
      tx, rx = (target.tx, target.subaddress), (target.rx, target.subaddress)
      if ((target.bus, target.obd) != (bus, obd) or type(item["emissions"]) is not bool or tx in requests or rx in replies):
        raise ValueError("Ambiguous cached diagnostic address")
      requests.add(tx)
      replies.add(rx)
      all_targets.add(target)
  if identity_target not in all_targets:
    raise ValueError("Vehicle identity missing from cached modules")
  return data


def read_vin_hash(transport, target, request):
  def decode(data):
    data = data.rstrip(b"\x00")
    if request == b"\x22\xf1\x90" and data[:1] == b"\x11":
      data = data[1:]
    vin = data.decode("ascii")
    if re.fullmatch(r"[A-HJ-NPR-Z0-9]{17}", vin) is None or vin == "0" * 17:
      raise ValueError("Invalid VIN")
    return hashlib.sha256(data).hexdigest()

  query = Reader(transport, target).read("cache_vehicle_identity", request, VIN_REQUESTS[request], decode)
  return query.get("data") if query["outcome"] == "ok" else None


def find_vehicle_identity(panda, targets, timeout, deadline):
  # Only query discovered endpoints, with a small total budget. Never trust a stale CarVin param.
  wire = PandaTransport(panda, timeout, min(deadline, time.monotonic() + 5))
  for target in sorted(targets, key=lambda t: (t.tx not in (0x7E0, 0x18DA10F1), target_key(t))):
    for request in VIN_REQUESTS:
      if time.monotonic() >= wire.deadline:
        return None
      vin_hash = read_vin_hash(wire, target, request)
      if vin_hash:
        return {"target": target.as_dict(), "request": request.hex(), "vin_hash": vin_hash}
  return None


def save_module_cache(path, data):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, prefix="modules-", suffix=".tmp", delete=False) as stream:
      temporary = Path(stream.name)
      json.dump(data, stream, indent=2)
      stream.write("\n")
    temporary.replace(path)
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)


def scan(panda, args, dataset, known_targets, safety_model):
  started = time.monotonic()
  deadline = started + args.scan_timeout
  report = {"schema_version": SCHEMA_VERSION, "report_kind": "technical_evidence",
            "started_at": datetime.now(UTC).isoformat(), "status": "partial", "setup_error": True,
            "coverage": "best_effort", "vehicle_coverage_complete": False, "ecus": [], "discovery": [], "errors": [], "warnings": [],
            "description_database": {key: dataset[key] for key in ("source", "revision", "license") if key in dataset},
            "scope": "Classic CAN OBD-II and UDS; no K-line, J1850, DoIP, security unlocks, or ECU writes."}
  routes = selected_routes(args)
  report["routes"] = [{"bus": bus, "obd_multiplexing": obd, "outcome": "not_scanned"} for bus, obd in routes]
  cache_path = module_cache_path()
  cache, identity, refreshed = None, None, []
  report["cache"] = {"fast_requested": args.fast, "path": str(cache_path), "routes_reused": 0, "updated": False}
  try:
    report["preflight"] = hardware_preflight(panda)
    for check in report["preflight"]["checks"]:
      print(f"Preflight [{check['status']}]: {check['message']}", file=sys.stderr)
      if check["status"] == "fail":
        report["errors"].append(check["message"])
      elif check["status"] == "warning":
        report["warnings"].append(check["message"])
    if not report["preflight"]["ready"]:
      report.update(status="failed", setup_error=True, elapsed_seconds=round(time.monotonic() - started, 3), deadline_reached=False)
      return report
    transport = PandaTransport(panda, args.timeout, deadline)
    report.pop("setup_error")
    try:
      cache = load_module_cache(cache_path)
    except FileNotFoundError:
      if args.fast:
        report["warnings"].append("No module cache yet; running discovery.")
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
      report["warnings"].append(f"Module cache unavailable; running discovery: {e}")
    for route in report["routes"]:
      if time.monotonic() >= deadline:
        break
      bus, obd = route["bus"], route["obd_multiplexing"]
      route["outcome"] = "incomplete"
      panda.set_safety_mode(safety_model.elm327, 0 if obd else 1)
      panda.can_clear(0xFFFF)
      if obd:
        route["preflight"] = check_obd_link(panda, deadline)
        check = route["preflight"]
        print(f"Preflight [{check['status']}]: {check['message']}", file=sys.stderr)
        if check["status"] != "pass":
          report["warnings"].append(check["message"])
        if check["status"] == "fail":
          route["outcome"] = "skipped"
          continue
      if args.fast and cache and identity is None and args.rx_addr is None:
        saved_identity = cache["identity"]
        target = cached_target(saved_identity["target"])
        if (target.bus, target.obd) == (bus, obd):
          vin_hash = read_vin_hash(transport, target, bytes.fromhex(saved_identity["request"]))
          if vin_hash == saved_identity["vin_hash"]:
            identity = saved_identity
          else:
            report["warnings"].append("Cached vehicle could not be verified or has changed; running discovery.")
      cached_route = next((r for r in cache["routes"] if (r["bus"], r["obd_multiplexing"]) == (bus, obd)), None) if cache else None
      use_cache = args.fast and identity is not None and cache is not None and identity["vin_hash"] == cache["identity"]["vin_hash"]
      items = cached_route["targets"] if cached_route is not None else []
      cached_found = {cached_target(t) for t in items if args.addr is None or
                      (int(t["tx_address"], 16) == args.addr and (args.subaddress is None or t["subaddress"] == args.subaddress))}
      if args.rx_addr is not None:
        route["source"] = "explicit"
        found = {Target(args.addr, args.rx_addr, bus, obd, args.subaddress)}
        emissions = found
      elif use_cache and cached_route is not None and (args.addr is None or cached_found):
        route.update(source="cache", cached_at=cached_route["scanned_at"])
        found = cached_found
        emissions = {cached_target(t) for t in items if t["emissions"]} & found
        report["cache"]["routes_reused"] += 1
        print(f"Using {len(found)} cached modules on bus {bus} ({'OBD port' if obd else 'harness'}); reading fresh faults…", file=sys.stderr)
      else:
        route["source"] = "discovery"
        probes = selected_probes(args, known_targets, bus, obd)
        print(f"Learning ECU reply addresses on bus {bus} ({'OBD port' if obd else 'harness'}; {len(probes)} probes)…", file=sys.stderr)
        found, emissions, discovery = discover(panda, probes, bus, obd, deadline, args.probe_timeout, args.timeout)
        report["discovery"].append(discovery)
        if discovery["unconfirmed"] or discovery["ambiguous"]:
          report["warnings"].append(f"Bus {bus} ({'OBD port' if obd else 'harness'}): " +
                                    f"{len(discovery['unconfirmed'])} unconfirmed and {len(discovery['ambiguous'])} ambiguous reply pairs " +
                                    "were not treated as identified ECUs. Use --evidence FILE to save their addresses and raw replies.")
      for target in sorted(found, key=target_key):
        if time.monotonic() >= deadline:
          report["ecus"].append({**target.as_dict(), "dtc_read": False, "codes": [], "queries": [], "outcome": "not_queried"})
          continue
        print(f"Reading bus {bus} ECU {target.tx:#x}…", file=sys.stderr)
        # Standard emissions addresses may support modes 03/07/0A even without a PID 00 response.
        use_obd = target in emissions or 0x7E0 <= target.tx <= 0x7E7
        result = query_ecu(transport, target, dataset.get("entries", {}), args.details, args.max_details, use_obd)
        result["identity_candidates"] = known_targets.get(target, [])  # Address matches are not vehicle identification.
        report["ecus"].append(result)
        if result.get("interrupted"):
          raise KeyboardInterrupt
        if result.get("error"):
          raise RuntimeError(result["error"])
      if time.monotonic() < deadline:
        route["outcome"] = "finished"
        if (route["source"] == "discovery" and args.addr is None and not discovery["not_probed"] and
            not discovery["unconfirmed"] and not discovery["ambiguous"]):
          if identity is None:
            identity = find_vehicle_identity(panda, found, args.timeout, deadline)
          refreshed.append({"bus": bus, "obd_multiplexing": obd, "scanned_at": report["started_at"],
                            "targets": [{**target.as_dict(), "emissions": target in emissions} for target in sorted(found, key=target_key)]})
  except KeyboardInterrupt:
    report["errors"].append("Scan interrupted; retained completed ECU results")
  except Exception as e:
    report["errors"].append(f"{type(e).__name__}: {e}")
  report["deadline_reached"] = time.monotonic() >= deadline
  # Interrupted, targeted, and deadline-limited scans must not replace a usable inventory.
  if refreshed and identity and not report["errors"] and not report["deadline_reached"]:
    previous = cache["routes"] if cache and cache["identity"]["vin_hash"] == identity["vin_hash"] else []
    merged = {(r["bus"], r["obd_multiplexing"]): r for r in previous + refreshed}
    try:
      save_module_cache(cache_path, {"version": 1, "identity": identity, "routes": list(merged.values())})
      report["cache"]["updated"] = True
    except KeyboardInterrupt:
      report["errors"].append("Cache update interrupted; retained completed ECU results")
    except OSError as e:
      report["warnings"].append(f"Could not save module cache: {e}")
  if refreshed and identity is None:
    report["warnings"].append("No live VIN available; module cache not updated. --fast requires a verified vehicle and will rediscover it.")
  if report["cache"]["routes_reused"]:
    report["warnings"].append("Fast scan checks cached modules only on reused routes; run without --fast to discover new or previously missed modules.")
  report["elapsed_seconds"] = round(time.monotonic() - started, 3)
  if not any(ecu["dtc_read"] for ecu in report["ecus"]):
    report["status"] = "failed"
  return report


def diagnosis_report(report):
  """One model/human-facing view; the original report remains intact as technical evidence."""
  if report.get("report_kind") == "diagnosis":
    return report
  result = {key: report[key] for key in ("started_at", "status", "setup_error", "coverage", "vehicle_coverage_complete", "scope",
                                        "errors", "warnings", "elapsed_seconds", "deadline_reached", "description_database", "cache",
                                        "evidence_file", "replay") if key in report}
  result.update(schema_version=SCHEMA_VERSION, report_kind="diagnosis", ecus=[],
                interpretation="Fault/history records are not a count of active problems. Missing data does not mean healthy.",
                reference_notice="OBDex entries are third-party reference material, not vehicle findings. " +
                                 "Possible causes and repair estimates are not diagnoses; flags.mil is not the vehicle's lamp state.")
  if "preflight" in report:
    result["preflight"] = {key: report["preflight"][key] for key in ("ready", "checks")}
  result["routes"] = []
  for route in report.get("routes", []):
    item = {key: value for key, value in route.items() if key != "preflight"}
    if "preflight" in route:
      item["preflight"] = {key: route["preflight"][key] for key in ("status", "message", "comma_power_present")}
    result["routes"].append(item)
  result["discovery"] = [{"bus": route["bus"], "obd_multiplexing": route["obd_multiplexing"],
                          "probe_count": route["probe_count"], "not_probed": route["not_probed"],
                          **{f"{key}_count": len(route[key]) for key in ("unanswered", "unconfirmed", "ambiguous")}}
                         for route in report.get("discovery", [])]
  for ecu in report["ecus"]:
    item = {key: ecu[key] for key in ("bus", "obd_multiplexing", "tx_address", "rx_address", "subaddress", "identity", "dtc_read",
                                     "outcome", "error", "interrupted") if key in ecu}
    queries = ecu.get("queries", [])
    item["read_results"] = [{key: query[key] for key in ("name", "outcome", "error") if key in query}
                            for query in queries if query["name"] in DTC_QUERIES]
    successful = {query["name"]: query["data"] for query in queries if query["outcome"] == "ok"}
    if "obd_monitor_status" in successful:
      item["emissions_status"] = {"source": "ecu", "timing": "at_scan", **{
        key: successful["obd_monitor_status"][key] for key in ("mil_on", "stored_dtc_count")}}
    measurements = {name: {key: successful[f"freeze_{name}"][key] for key in ("value", "unit")}
                    for name, _ in FREEZE_PIDS.values() if f"freeze_{name}" in successful}
    item["codes"] = []
    for code in ecu["codes"]:
      fault = {key: code[key] for key in ("protocol", "code", "display_code", "failure_type", "status", "lookup", "search") if key in code}
      fault["status_summary"] = " ".join(STATUS_MEANINGS[flag] for flag in code["status"] if flag in STATUS_MEANINGS) or "Status unknown."
      # A generic OBD frame cannot safely be attached to a UDS code or a different ECU.
      if code["protocol"] == "obd" and code["code"] == successful.get("freeze_dtc") and code["code"] != "P0000" and measurements:
        fault["freeze_frame"] = {"source": "ecu", "historical": True, "frame": 0, "code": code["code"], "measurements": measurements}
      item["codes"].append(fault)
    result["ecus"].append(item)
  result["summary"] = {"module_endpoints_listed": len(result["ecus"]),
                       "modules_with_dtc_data": sum(ecu["dtc_read"] for ecu in result["ecus"]),
                       "modules_without_dtc_data": sum(not ecu["dtc_read"] for ecu in result["ecus"]),
                       "fault_history_records": sum(len(ecu["codes"]) for ecu in result["ecus"])}
  return result


def print_report(report):
  report = diagnosis_report(report)
  print(f"Diagnostic scan: {report['status']} (vehicle-wide coverage is not verified)")
  print(report["interpretation"])
  print(report["reference_notice"])
  summary = report["summary"]
  print(f"{summary['fault_history_records']} fault/history records; " +
        f"{summary['modules_with_dtc_data']}/{summary['module_endpoints_listed']} listed module endpoints returned DTC data.")
  for ecu in report["ecus"]:
    route = "OBD port" if ecu["obd_multiplexing"] else "harness"
    print(f"\nBus {ecu['bus']} / {route} / {ecu['tx_address']} → {ecu['rx_address']} / subaddress {ecu['subaddress']}")
    if ecu.get("identity"):
      print("  ECU: " + " / ".join(ecu["identity"][name] for name in ("component", "part_number", "software_version") if ecu["identity"].get(name)))
    if not ecu["dtc_read"]:
      print("  DTCs unavailable; this is not a clean bill of health.")
    elif not ecu["codes"]:
      print("  No fault/history records returned by successful queries.")
    if "emissions_status" in ecu:
      print(f"  Check-engine light requested at scan: {'yes' if ecu['emissions_status']['mil_on'] else 'no'} (ECU-reported).")
    for code in ecu["codes"]:
      failure = f"-{code['failure_type'][2:]}" if "failure_type" in code else ""
      print(f"  {code.get('display_code', code['code'] + failure)} [{code['protocol']}] {', '.join(code['status']) or 'no supported status bits set'}")
      print(f"    {code['status_summary']}")
      if "freeze_frame" in code:
        values = ", ".join(f"{name.replace('_', ' ')}: {value['value']:g} {value['unit']}"
                           for name, value in code["freeze_frame"]["measurements"].items())
        print(f"    Historical freeze frame for {code['code']} (not live): {values}")
      if "lookup" in code:
        print("    OBDex reference (possible causes/estimates, not confirmed vehicle findings):")
        print("\n".join("      " + line for line in json.dumps(code["lookup"]["entry"], indent=2, ensure_ascii=False).splitlines()))
      elif "search" in code:
        print(f"    Search: {code['search']['query']}")
    for query in ecu["read_results"]:
      if query["outcome"] != "ok":
        print(f"  {query['name']}: {query['outcome']} — {query.get('error', '')}")
    if ecu.get("error"):
      print(f"  Error: {ecu['error']}")
  for error in report["errors"]:
    print(f"Error: {error}")
  if report.get("deadline_reached"):
    print("Scan deadline reached; remaining queries were not completed.")
  if not report["ecus"]:
    print("No ECU scan performed; resolve the setup errors above." if report.get("setup_error") else
          "No ECU data retrieved. Check the preflight results, selected bus, and gateway access.")
  for warning in report.get("warnings", []):
    print(f"Warning: {warning}")
  if "evidence_file" in report:
    print(f"Technical evidence: {report['evidence_file']}")


def positive_seconds(value):
  number = float(value)
  if not math.isfinite(number) or number <= 0:
    raise argparse.ArgumentTypeError("must be finite and positive")
  return number


def make_parser():
  parser = argparse.ArgumentParser(description=__doc__,
                                   epilog="Exit 0: some DTC data read (coverage may be partial). Exit 1: no DTC data. Exit 2: setup error.")
  parser.add_argument("--addr", type=lambda value: int(value, 0), help="target a physical ECU address instead of auto-scanning")
  parser.add_argument("--rx-addr", type=lambda value: int(value, 0), help="explicit reply address, e.g. VW tx + 0x6a")
  parser.add_argument("--subaddress", type=lambda value: int(value, 0), help="ISO-TP subaddress")
  parser.add_argument("--broad", action="store_true", help="scan harness routes as well as OBD (default: OBD only)")
  parser.add_argument("--fast", action="store_true", help="reuse vehicle-verified cached modules; discover routes without a usable cache")
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), action="append",
                      help="override buses; repeat to select several (default: 1, or 1/0/2 with --broad)")
  parser.add_argument("--obd", choices=("auto", "on", "off"), help="override bus 1 routing: auto tries both (default: on, or auto with --broad)")
  parser.add_argument("--serial", help="Panda serial (required if several are connected)")
  parser.add_argument("--details", action="store_true", help="also read raw UDS details and identifiers on fault-free modules (save with --evidence)")
  parser.add_argument("--max-details", type=int, default=16, help="maximum UDS faults per ECU for detail retrieval (default: 16)")
  parser.add_argument("--timeout", type=positive_seconds, default=1.0, help="absolute per-request timeout, including response-pending (seconds)")
  parser.add_argument("--probe-timeout", type=positive_seconds, default=0.1, help="listen window per discovery probe in seconds (default: 0.1)")
  parser.add_argument("--scan-timeout", type=positive_seconds, default=600.0, help="total query/discovery budget in seconds (default: 600)")
  parser.add_argument("--json", action="store_true", help="emit one JSON report on stdout; progress goes to stderr")
  parser.add_argument("--evidence", type=Path, metavar="FILE", help="save full technical evidence separately as JSON; refuses to overwrite a file")
  parser.add_argument("--debug", action="store_true", help="ISO-TP logging to stderr")
  return parser


def check_pandad():
  try:
    check = subprocess.run(["pgrep", "-x", "pandad"], capture_output=True, check=False)
  except FileNotFoundError as e:
    raise RuntimeError("pgrep unavailable; cannot verify exclusive Panda access") from e
  if check.returncode == 0:
    raise RuntimeError("pandad is running. Park, stop openpilot (tmux kill-session -t comma), then retry with ignition on.")
  if check.returncode != 1:
    raise RuntimeError("Could not check whether pandad is running")


def main(argv=None):
  parser = make_parser()
  args = parser.parse_args(argv)
  if args.addr is None and (args.rx_addr is not None or args.subaddress is not None):
    parser.error("--rx-addr and --subaddress require --addr")
  if args.addr is not None and not valid_tx(args.addr):
    parser.error("--addr must be a physical diagnostic address allowed by Panda ELM327 safety")
  if args.rx_addr is not None and not 0 <= args.rx_addr <= 0x1FFFFFFF:
    parser.error("--rx-addr must be a CAN address")
  if args.subaddress is not None and not 0 <= args.subaddress <= 255:
    parser.error("--subaddress must fit in one byte")
  if args.max_details < 0:
    parser.error("--max-details must be nonnegative")

  panda = None
  report = None
  warnings = []
  try:
    check_pandad()
    from opendbc.car.carlog import carlog
    from opendbc.car.structs import CarParams
    from panda import Panda
    if args.debug:
      carlog.setLevel("DEBUG")
    try:
      dataset = load_dataset()
    except (OSError, ValueError, EOFError) as e:
      dataset = {}
      warnings.append(f"Offline descriptions unavailable: {e}")
    try:
      known_targets = load_known_targets()
      if not known_targets:
        warnings.append("No brand hints loaded; generic discovery remains available")
    except (ImportError, OSError) as e:
      known_targets = {}
      warnings.append(f"Brand hints unavailable; using generic discovery: {e}")
    serials = Panda.list()
    if args.serial is None and len(serials) > 1:
      raise RuntimeError(f"Multiple pandas connected; choose --serial from {serials}")
    # cli=False prevents Panda from prompting or printing into JSON stdout.
    panda = Panda(serial=args.serial, cli=False)
    report = scan(panda, args, dataset, known_targets, CarParams.SafetyModel)
  except Exception as e:
    report = {"schema_version": SCHEMA_VERSION, "report_kind": "technical_evidence", "status": "failed", "vehicle_coverage_complete": False,
              "ecus": [], "errors": [f"{type(e).__name__}: {e}"], "setup_error": True}
  finally:
    if panda is not None:
      try:
        panda.set_safety_mode(CarParams.SafetyModel.noOutput)
      except Exception as e:
        warnings.append(f"Failed to restore Panda no-output mode: {e}")
      finally:
        try:
          panda.close()
        except Exception as e:
          warnings.append(f"Failed to close Panda: {e}")
  report.setdefault("warnings", []).extend(warnings)
  if args.evidence is not None:
    try:
      with args.evidence.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
      report["evidence_file"] = str(args.evidence.resolve())
    except OSError as e:
      report["warnings"].append(f"Could not save technical evidence to {args.evidence}: {e}")
  if args.json:
    print(json.dumps(diagnosis_report(report), indent=2, ensure_ascii=False))
  else:
    print_report(report)
  return 2 if report.get("setup_error") else (1 if report["status"] == "failed" else 0)


if __name__ == "__main__":
  sys.exit(main())
