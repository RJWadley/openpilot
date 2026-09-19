#!/usr/bin/env python3
"""Read OBD-II/UDS faults on a parked vehicle with openpilot stopped. See diagnose.md."""
import argparse
import json
import math
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, UTC
from functools import partial
from pathlib import Path


DATASET = Path(__file__).with_name("data") / "obdex.json"
STATUS_BITS = (
  "test_failed", "test_failed_this_operation_cycle", "pending", "confirmed",
  "test_not_completed_since_last_clear", "test_failed_since_last_clear",
  "test_not_completed_this_operation_cycle", "warning_indicator_requested",
)
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


def query_ecu(transport, target, labels, details=False, max_details=16, obd=False):
  reader = Reader(transport, target)
  result = {**target.as_dict(), "codes": [], "queries": reader.queries, "dtc_read": False}
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
      query["data"] = records
      result["codes"].extend(records)
      result["uds_status_availability"] = bytes.fromhex(query["responses"][-1])[2]

    if obd and target.subaddress is None:
      for mode, status in OBD_MODES.items():
        query = reader.read(f"obd_{status}", bytes([mode]), bytes([mode + 0x40]), partial(parse_obd_codes, status=status))
        if query["outcome"] == "ok":
          result["dtc_read"] = True
          result["codes"].extend(query["data"])
      reader.read("obd_monitor_status", b"\x01\x01", b"\x41\x01", parse_monitor_status)

    for record in result["codes"]:
      if record["code"] in labels:
        record["lookup"] = {"source": "OBDex", "title": labels[record["code"]]}

    if details and result["dtc_read"]:
      for did, name in IDENTIFIERS.items():
        request = b"\x22" + did.to_bytes(2, "big")
        reader.read(name, request, b"\x62" + request[1:], lambda data: data.decode("utf-8", errors="replace").rstrip("\x00"))
      uds_records = [code for code in result["codes"] if code["protocol"] == "uds"]
      result["details_omitted"] = max(0, len(uds_records) - max_details)
      for code in uds_records[:max_details]:
        raw = bytes.fromhex(code["raw_dtc"])
        for subfunction, name in ((4, "snapshot"), (6, "extended_data")):
          # Keep the ECU-specific record body raw; the response prefix validates the echoed DTC.
          reader.read(f"{name}_{code['raw_dtc']}", bytes([0x19, subfunction]) + raw + b"\xff",
                      bytes([0x59, subfunction]) + raw, parse_uds_detail)
      if obd:
        read_freeze_frame(reader)
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


def generic_targets(bus, obd):
  addresses = [address for address in range(0x700, 0x7F8) if address != 0x7DF]
  addresses += [0x18DA00F1 | (node << 8) for node in range(256) if node != 0xF1]
  return {Target(address, rx_address(address), bus, obd) for address in addresses}


def selected_targets(args, known_targets, bus, obd):
  if args.rx_addr is not None:
    return {Target(args.addr, args.rx_addr, bus, obd, args.subaddress)}
  targets = {target for target in known_targets if (target.tx, target.bus, target.obd) == (args.addr, bus, obd)
             and (args.subaddress is None or target.subaddress == args.subaddress)}
  if args.addr <= 0x7F7 or args.addr > 0x7FF:
    targets.add(Target(args.addr, rx_address(args.addr), bus, obd, args.subaddress))
  return targets


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


def discover(panda, candidates, bus, obd, deadline, probe_wait=0.15):
  """Batched single-frame discovery. DTC reads then validate each physical address independently."""
  by_response = defaultdict(set)
  probes = {}
  for target in sorted(candidates, key=target_key):
    by_response[target.rx].add(target)
    probes[(target.tx, target.subaddress)] = target
  found, emissions, evidence = set(), set(), []
  sent = set()

  def collect():
    for address, data, rx_bus in panda.can_recv():
      if rx_bus != bus:
        continue
      payload = frame_payload(data)
      if payload is not None and len(payload) == 6 and payload[:2] == b"\x41\x00":
        if 0x7E8 <= address <= 0x7EF:
          target = Target(address - 8, address, bus, obd)
        elif address & 0x1FFFFF00 == 0x18DAF100:
          target = Target(0x18DA00F1 | ((address & 0xFF) << 8), address, bus, obd)
        else:
          continue
        emissions.add(target)
        found.add(target)
        evidence.append({**target.as_dict(), "request_address": hex(0x7DF if address <= 0x7FF else 0x18DB33F1),
                         "request": "0100", "response": payload.hex()})
      for target in by_response.get(address, ()):
        if (target.tx, target.subaddress) not in sent:
          continue
        payload = frame_payload(data, target.subaddress)
        if payload == b"\x7e\x00" or (payload is not None and len(payload) == 3 and payload[:2] == b"\x7f\x3e"):
          found.add(target)
          evidence.append({**target.as_dict(), "request": "3e00", "response": payload.hex()})

  # Functional OBD discovers all responders; no first-responder-only UdsClient behavior.
  for address in (0x7DF, 0x18DB33F1):
    if time.monotonic() >= deadline:
      break
    panda.can_send(address, single_frame(b"\x01\x00"), bus, timeout=100)
  ordered = list(probes.values())
  for start in range(0, len(ordered), 24):
    if time.monotonic() >= deadline:
      break
    for target in ordered[start:start + 24]:
      if time.monotonic() >= deadline:
        break
      panda.can_send(target.tx, single_frame(b"\x3e\x00", target.subaddress), bus, timeout=100)
      sent.add((target.tx, target.subaddress))
      collect()
      time.sleep(0.002)
    until = min(deadline, time.monotonic() + probe_wait)
    while time.monotonic() < until:
      collect()
      time.sleep(0.002)
  return found, emissions, {"bus": bus, "obd_multiplexing": obd, "probe_count": len(sent), "responses": evidence,
                             "unanswered": [t.as_dict() for t in sorted(candidates - found, key=target_key)
                                            if (t.tx, t.subaddress) in sent],
                             "not_probed": sum((t.tx, t.subaddress) not in sent for t in candidates)}


def scan(panda, args, dataset, known_targets, safety_model):
  started = time.monotonic()
  deadline = started + args.scan_timeout
  report = {"schema_version": 1, "started_at": datetime.now(UTC).isoformat(), "status": "partial",
            "coverage": "best_effort", "vehicle_coverage_complete": False, "ecus": [], "discovery": [], "errors": [],
            "description_database": {key: dataset[key] for key in ("source", "revision", "license") if key in dataset},
            "scope": "Classic CAN OBD-II and UDS; no K-line, J1850, DoIP, security unlocks, or ECU writes."}
  buses = args.bus if args.bus is not None else ([1] if args.addr is not None else [1, 0, 2])
  routes = [(bus, mux) for bus in buses for mux in ((True, False) if bus == 1 and args.obd == "auto" else
                                                  (bus == 1 and args.obd != "off",))]
  report["routes"] = [{"bus": bus, "obd_multiplexing": obd, "outcome": "not_scanned"} for bus, obd in routes]
  transport = PandaTransport(panda, args.timeout, deadline)
  try:
    for route in report["routes"]:
      if time.monotonic() >= deadline:
        break
      bus, obd = route["bus"], route["obd_multiplexing"]
      route["outcome"] = "incomplete"
      panda.set_safety_mode(safety_model.elm327, 0 if obd else 1)
      panda.can_clear(0xFFFF)
      if args.addr is not None:
        found = selected_targets(args, known_targets, bus, obd)
        emissions = found
        if not found:
          report["errors"].append(f"No reply address known for {args.addr:#x}; specify --rx-addr")
      else:
        candidates = generic_targets(bus, obd) | {target for target in known_targets if (target.bus, target.obd) == (bus, obd)}
        print(f"Discovering ECUs on bus {bus} ({'OBD port' if obd else 'harness'})…", file=sys.stderr)
        found, emissions, discovery = discover(panda, candidates, bus, obd, deadline)
        report["discovery"].append(discovery)
      for target in sorted(found, key=target_key):
        if time.monotonic() >= deadline:
          report["ecus"].append({**target.as_dict(), "dtc_read": False, "codes": [], "queries": [], "outcome": "not_queried"})
          continue
        print(f"Reading bus {bus} ECU {target.tx:#x}…", file=sys.stderr)
        # Standard emissions addresses may support modes 03/07/0A even without a PID 00 response.
        use_obd = target in emissions or 0x7E0 <= target.tx <= 0x7E7
        result = query_ecu(transport, target, dataset.get("labels", {}), args.details, args.max_details, use_obd)
        result["identity_candidates"] = known_targets.get(target, [])  # Address matches are not vehicle identification.
        report["ecus"].append(result)
        if result.get("interrupted"):
          raise KeyboardInterrupt
        if result.get("error"):
          raise RuntimeError(result["error"])
      if time.monotonic() < deadline:
        route["outcome"] = "finished"
  except KeyboardInterrupt:
    report["errors"].append("Scan interrupted; retained completed ECU results")
  except Exception as e:
    report["errors"].append(f"{type(e).__name__}: {e}")
  report["deadline_reached"] = time.monotonic() >= deadline
  report["elapsed_seconds"] = round(time.monotonic() - started, 3)
  if not any(ecu["dtc_read"] for ecu in report["ecus"]):
    report["status"] = "failed"
  return report


def print_report(report):
  print(f"Diagnostic scan: {report['status']} (vehicle-wide coverage is not verified)")
  for ecu in report["ecus"]:
    route = "OBD port" if ecu["obd_multiplexing"] else "harness"
    print(f"\nBus {ecu['bus']} / {route} / {ecu['tx_address']} → {ecu['rx_address']} / subaddress {ecu['subaddress']}")
    if ecu.get("identity_candidates"):
      print("  Address hints: " + ", ".join(ecu["identity_candidates"]))
    if not ecu["dtc_read"]:
      print("  DTCs unavailable; this is not a clean bill of health.")
    elif not ecu["codes"]:
      print("  No codes returned by successful queries.")
    for code in ecu["codes"]:
      failure = f"-{code['failure_type'][2:]}" if "failure_type" in code else ""
      print(f"  {code['code']}{failure} [{code['protocol']}] {', '.join(code['status']) or 'no supported status bits set'}")
      if "lookup" in code:
        print(f"    OBDex: {code['lookup']['title']}")
    for query in ecu["queries"]:
      if query["outcome"] != "ok":
        print(f"  {query['name']}: {query['outcome']} — {query.get('error', '')}")
      elif query["name"] not in ("uds_codes", "uds_count", "obd_stored", "obd_pending", "obd_permanent"):
        print(f"  {query['name']}: {json.dumps(query['data'], ensure_ascii=False)}")
    if ecu.get("details_omitted"):
      print(f"  Details omitted for {ecu['details_omitted']} codes (--max-details).")
  for error in report["errors"]:
    print(f"Error: {error}")
  if report.get("deadline_reached"):
    print("Scan deadline reached; remaining queries were not completed.")
  if not report["ecus"]:
    print("No ECUs responded. Check ignition, wiring, selected bus, and gateway access.")


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
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), action="append", help="repeat to select buses; default: all, or bus 1 for targeting")
  parser.add_argument("--obd", choices=("auto", "on", "off"), default="auto", help="bus 1 OBD multiplexing; auto tries both routes")
  parser.add_argument("--serial", help="Panda serial (required if several are connected)")
  parser.add_argument("--details", action="store_true", help="read ECU identifiers, UDS raw snapshots/extended records and OBD freeze frame zero")
  parser.add_argument("--max-details", type=int, default=16, help="maximum UDS codes per ECU for detail retrieval (default: 16)")
  parser.add_argument("--timeout", type=positive_seconds, default=1.0, help="absolute per-request timeout, including response-pending (seconds)")
  parser.add_argument("--scan-timeout", type=positive_seconds, default=120.0, help="total query/discovery budget in seconds")
  parser.add_argument("--json", action="store_true", help="emit one JSON report on stdout; progress goes to stderr")
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
      dataset = json.loads(DATASET.read_text())
    except (OSError, ValueError) as e:
      dataset = {}
      warnings.append(f"Offline descriptions unavailable: {e}")
    try:
      known_targets = load_known_targets()
      if not known_targets:
        warnings.append("No brand addresses loaded; using generic addresses only")
    except (ImportError, OSError) as e:
      known_targets = {}
      warnings.append(f"Brand addressing unavailable; using generic addresses: {e}")
    serials = Panda.list()
    if args.serial is None and len(serials) > 1:
      raise RuntimeError(f"Multiple pandas connected; choose --serial from {serials}")
    # cli=False prevents Panda from prompting or printing into JSON stdout.
    panda = Panda(serial=args.serial, cli=False)
    report = scan(panda, args, dataset, known_targets, CarParams.SafetyModel)
  except Exception as e:
    report = {"schema_version": 1, "status": "failed", "vehicle_coverage_complete": False,
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
  report["warnings"] = warnings
  if args.json:
    print(json.dumps(report, indent=2, ensure_ascii=False))
  else:
    print_report(report)
    for warning in warnings:
      print(f"Warning: {warning}")
  return 2 if report.get("setup_error") else (1 if report["status"] == "failed" else 0)


if __name__ == "__main__":
  sys.exit(main())
