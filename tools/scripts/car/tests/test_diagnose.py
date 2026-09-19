import contextlib
import io
import json
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.structs import CarParams
from tools.scripts.car import diagnose as d


SAFETY = CarParams.SafetyModel


class FakePanda:
  """A CAN-level ECU simulator: real opendbc ISO-TP handles segmentation and flow control."""
  def __init__(self, ecus):
    self.ecus = ecus  # {Target: {request: response}}, with callable responses allowed
    self.queue = []
    self.pending = {}
    self.sent = []
    self.requests = []
    self.obd = True
    self.safety_modes = []
    self.closed = False
    self.internal = True
    self.health_data = {"ignition_line": True, "ignition_can": False, "car_harness_status": 1, "faults": 0, "voltage": 12000}
    self.can_health_data = {"bus_off": False, "total_error_cnt": 0, "last_error": "No error", "last_stored_error": "No error"}
    self.obd_disconnected = False

  def health(self):
    return dict(self.health_data)

  def is_internal(self):
    return self.internal

  def can_health(self, bus):
    return dict(self.can_health_data)

  def can_send(self, address, data, bus, timeout=0):
    self.sent.append((address, bytes(data), bus))
    if self.obd_disconnected and bus == 1 and self.obd:
      self.can_health_data.update(total_error_cnt=self.can_health_data["total_error_cnt"] + 1,
                                  last_error="AckError", last_stored_error="AckError")
      return
    for target, responses in self.ecus.items():
      if target.bus != bus or (bus == 1 and target.obd != self.obd):
        continue
      functional = address in (0x7DF, 0x18DB33F1)
      if address != target.tx and not functional:
        continue
      sub = target.subaddress
      offset = int(sub is not None)
      if data[offset] == 0x30:
        self.queue.extend(self.pending.pop(target, []))
        continue
      request = d.frame_payload(data, sub)
      if request is None:
        continue
      self.requests.append((target, request))
      if functional and (sub is not None or request not in responses):
        continue
      if functional and (address == 0x7DF) != (target.tx <= 0x7FF):
        continue
      response = responses.get(request, b"\x7e\x00" if request == b"\x3e\x00" else bytes([0x7F, request[0], 0x11]))
      if callable(response):
        response = response(request)
      if response is None:
        continue
      for payload in response if isinstance(response, list) else [response]:
        prefix = b"" if sub is None else bytes([sub])
        capacity = 7 - offset
        if len(payload) <= capacity:
          self.queue.append((target.rx, d.single_frame(payload, sub), bus))
        else:
          first = prefix + bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:capacity - 1]
          self.queue.append((target.rx, first.ljust(8, b"\x00"), bus))
          tail = payload[capacity - 1:]
          self.pending[target] = [(target.rx, (prefix + bytes([0x20 | (i & 15)]) + tail[j:j + capacity]).ljust(8, b"\x00"), bus)
                                  for i, j in enumerate(range(0, len(tail), capacity), 1)]

  def can_recv(self):
    result, self.queue = self.queue, []
    return result

  def can_clear(self, bus):
    self.queue.clear()

  def set_safety_mode(self, mode, param=0):
    self.safety_modes.append((mode, param))
    self.obd = param == 0

  def close(self):
    self.closed = True


def transport(panda, timeout=0.02):
  return d.PandaTransport(panda, timeout, time.monotonic() + 5)


class TestDecoding(unittest.TestCase):
  def test_obd_count_and_status(self):
    codes = d.parse_obd_codes(bytes.fromhex("02 0301 c123 0000"), "pending")
    self.assertEqual([code["code"] for code in codes], ["P0301", "U0123"])
    self.assertEqual(codes[0]["status"], ["pending"])
    self.assertEqual(d.parse_obd_codes(b"\x00", "stored"), [])

  def test_obd_invalid_count_never_becomes_empty_success(self):
    for raw in (b"", bytes.fromhex("02 0301"), bytes.fromhex("00 0301"), bytes.fromhex("01 0000")):
      with self.subTest(raw=raw), self.assertRaises(ValueError):
        d.parse_obd_codes(raw, "stored")

  def test_code_families_and_hex_digits(self):
    for raw, expected in (("0301", "P0301"), ("4123", "C0123"), ("8001", "B0001"), ("cabc", "U0ABC"), ("3abc", "P3ABC")):
      self.assertEqual(d.format_obd_code(bytes.fromhex(raw)), expected)

  def test_uds_format_and_failure_type(self):
    raw = bytes.fromhex("ff 800113 89 030100 08")
    codes = d.parse_uds_codes(raw, 4)
    self.assertEqual(codes[0]["code"], "B0001")
    self.assertEqual(codes[0]["failure_type"], "0x13")
    self.assertEqual(codes[0]["status"], ["test_failed", "confirmed", "warning_indicator_requested"])
    for fmt in (None, 1, 2, 3, 255):
      self.assertEqual(d.parse_uds_codes(raw, fmt)[0]["code"], "0x800113")

  def test_uds_truncation_and_status_availability(self):
    with self.assertRaises(ValueError):
      d.parse_uds_codes(bytes.fromhex("ff 8001"), 4)
    self.assertEqual(d.parse_uds_codes(b"\xff", 4), [])
    self.assertEqual(d.parse_uds_codes(bytes.fromhex("08 030100 89"), 4)[0]["status"], ["confirmed"])

  def test_uds_count(self):
    self.assertEqual(d.parse_uds_count(bytes.fromhex("ff 04 0100"))["count"], 256)
    with self.assertRaises(ValueError):
      d.parse_uds_count(b"\xff\x04")

  def test_freeze_frame_units(self):
    self.assertEqual(d.parse_freeze_value(0x0C, bytes.fromhex("1f40"))["value"], 2000)
    self.assertEqual(d.parse_freeze_value(0x05, b"\x64")["value"], 60)
    self.assertEqual(d.parse_freeze_value(0x04, b"\xff")["value"], 100)
    self.assertEqual(d.parse_monitor_status(bytes.fromhex("82000000"))["stored_dtc_count"], 2)


class TestTransport(unittest.TestCase):
  def test_multiframe_obd_and_pending(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x03": [b"\x7f\x03\x78", bytes.fromhex("43 04 0301 0420 0123 c123")]}})
    result = d.Reader(transport(panda), target).read("stored", b"\x03", b"\x43", lambda data: d.parse_obd_codes(data, "stored"))
    self.assertEqual(result["outcome"], "ok")
    self.assertEqual(len(result["data"]), 4)
    self.assertEqual(result["responses"][0], "7f0378")
    self.assertTrue(any(frame[1][0] == 0x30 for frame in panda.sent))

  def test_subaddress_multiframe(self):
    target = d.Target(0x750, 0x758, subaddress=0x0F)
    payload = bytes.fromhex("59 02 ff 800113 89 030100 08")
    panda = FakePanda({target: {b"\x19\x02\xff": payload}})
    self.assertEqual(transport(panda).exchange(target, b"\x19\x02\xff"), payload)
    self.assertTrue(any(frame[1][:2] == b"\x0f\x30" for frame in panda.sent))

  def test_timeout_with_pending_is_bounded(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x03": b"\x7f\x03\x78"}})
    start = time.monotonic()
    result = d.Reader(transport(panda, 0.01), target).read("stored", b"\x03", b"\x43")
    self.assertEqual(result["outcome"], "timeout")
    self.assertEqual(result["responses"], ["7f0378"])
    self.assertLess(time.monotonic() - start, 0.5)

  def test_negative_and_malformed_responses(self):
    target = d.Target(0x7E0, 0x7E8)
    for raw, outcome in (("7f0311", "unsupported"), ("7f0333", "rejected"), ("7f1911", "malformed"), ("5902ff", "malformed")):
      with self.subTest(raw=raw):
        panda = FakePanda({target: {b"\x03": bytes.fromhex(raw)}})
        result = d.Reader(transport(panda), target).read("stored", b"\x03", b"\x43")
        self.assertEqual(result["outcome"], outcome)
        self.assertEqual(result["responses"], [raw])

  def test_write_requests_rejected_before_transmission(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({})
    wire = transport(panda)
    for request in (b"\x04", b"\x14\xff\xff\xff", b"\x27\x01", b"\x2e\xf1\x90", b"\x31\x01", b"\x10\x02"):
      with self.subTest(request=request), self.assertRaises(ValueError):
        wire.exchange(target, request)
    self.assertEqual(panda.sent, [])


class TestScanning(unittest.TestCase):
  def test_discovery_all_functional_responders_and_bus_filter(self):
    engine, transmission = d.Target(0x7E0, 0x7E8), d.Target(0x7E1, 0x7E9)
    extended = d.Target(0x18DA10F1, 0x18DAF110)
    vw = d.Target(0x715, 0x77F)
    sub = d.Target(0x750, 0x758, subaddress=0x0F)
    wrong_bus = d.Target(0x7E2, 0x7EA, bus=0, obd=False)
    mode01 = {b"\x01\x00": bytes.fromhex("4100be3fa813")}
    panda = FakePanda({engine: mode01, transmission: mode01, extended: mode01, vw: {}, sub: {}, wrong_bus: mode01})
    panda.queue.append((wrong_bus.rx, d.single_frame(bytes.fromhex("4100be3fa813")), 0))
    found, emissions, report = d.discover(panda, {engine, transmission, extended, vw, sub}, 1, True, time.monotonic() + 1, probe_wait=0)
    self.assertEqual(found, {engine, transmission, extended, vw, sub})
    self.assertEqual(emissions, {engine, transmission, extended})
    self.assertEqual(report["unanswered"], [])

  def test_scan_keeps_engine_when_airbag_cannot_read(self):
    engine, airbag = d.Target(0x7E0, 0x7E8), d.Target(0x715, 0x77F)
    panda = FakePanda({engine: {b"\x03": bytes.fromhex("43010301"), b"\x01\x00": bytes.fromhex("410000000000")}, airbag: {}})
    args = d.make_parser().parse_args(["--bus", "1", "--obd", "on", "--timeout", "0.02"])
    with patch.object(d, "generic_targets", return_value={engine, airbag}), contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {"labels": {"P0301": "Cylinder 1 Misfire Detected"}}, {}, SAFETY)
    self.assertEqual(report["status"], "partial")
    self.assertFalse(report["vehicle_coverage_complete"])
    results = {ecu["tx_address"]: ecu for ecu in report["ecus"]}
    self.assertFalse(results["0x715"]["dtc_read"])
    code = results["0x7e0"]["codes"][0]
    self.assertEqual(code["code"], "P0301")
    self.assertIn("lookup", code)
    json.dumps(report)

  def test_extended_session_only_on_demand_and_restored(self):
    target = d.Target(0x715, 0x77F)
    calls = []

    def codes(request):
      calls.append(request)
      return bytes.fromhex("7f197f" if len(calls) == 1 else "5902ff80011389")

    panda = FakePanda({target: {b"\x19\x02\xff": codes, b"\x10\x03": b"\x50\x03\x00\x32\x01\xf4",
                                b"\x10\x01": b"\x50\x01", b"\x19\x01\xff": bytes.fromhex("5901ff040001")}})
    result = d.query_ecu(transport(panda), target, {})
    self.assertEqual(result["codes"][0]["code"], "B0001")
    self.assertEqual(panda.requests[-1][1], b"\x10\x01")

  def test_unknown_format_preserves_code_without_false_label(self):
    target = d.Target(0x715, 0x77F)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff80011389")}})
    result = d.query_ecu(transport(panda), target, {"B0001": "Example"})
    self.assertEqual(result["codes"][0]["code"], "0x800113")
    self.assertNotIn("lookup", result["codes"][0])

  def test_details_preserve_raw_records_and_correct_obd_frame(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff03010089"),
                                b"\x19\x01\xff": bytes.fromhex("5901ff040001"),
                                b"\x19\x04\x03\x01\x00\xff": bytes.fromhex("59040301008901abcdef"),
                                b"\x02\x00\x00": bytes.fromhex("42000000100000"),
                                b"\x02\x02\x00": bytes.fromhex("4202000301"),
                                b"\x02\x0c\x00": bytes.fromhex("420c001f40")}})
    result = d.query_ecu(transport(panda), target, {}, details=True, obd=True)
    queries = {q["name"]: q for q in result["queries"]}
    self.assertEqual(queries["snapshot_030100"]["data"], {"status_byte": 0x89, "records_raw": "01abcdef"})
    self.assertEqual(queries["freeze_engine_speed"]["data"]["value"], 2000)
    self.assertEqual(queries["freeze_dtc"]["data"], "P0301")
    self.assertTrue(all(d.read_request(request) for _, request in panda.requests))

  def test_targeting_uses_known_rx_offset(self):
    target = d.Target(0x715, 0x77F)
    args = d.make_parser().parse_args(["--addr", "0x715"])
    self.assertIn(target, d.selected_targets(args, {target: ["volkswagen:srs"]}, 1, True))
    args.rx_addr = 0x77F
    self.assertEqual(d.selected_targets(args, {}, 1, True), {target})

  def test_no_response_is_failure_not_no_faults(self):
    args = d.make_parser().parse_args(["--addr", "0x7e0", "--obd", "on", "--timeout", "0.001"])
    with contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(FakePanda({}), args, {}, {}, SAFETY)
    self.assertEqual(report["status"], "failed")
    self.assertFalse(report["ecus"][0]["dtc_read"])

  def test_address_range_and_29_bit_mapping(self):
    self.assertEqual(d.rx_address(0x18DA10F1), 0x18DAF110)
    self.assertFalse(d.valid_tx(0x7DF))
    self.assertFalse(d.valid_tx(0x18DB33F1))
    self.assertTrue(d.valid_tx(0x24B))
    self.assertTrue(d.valid_tx(0x18DA10F1))

  def test_scan_deadline_retains_unqueried_routes(self):
    args = d.make_parser().parse_args(["--addr", "0x7e0", "--scan-timeout", "0.000001"])
    report = d.scan(FakePanda({}), args, {}, {}, SAFETY)
    self.assertTrue(report["deadline_reached"])
    self.assertEqual(report["status"], "failed")
    self.assertTrue(any(route["outcome"] == "not_scanned" for route in report["routes"]))

  def test_brand_profiles_load_real_vw_addresses(self):
    profiles = d.load_known_targets()
    self.assertIn("volkswagen:srs", profiles[d.Target(0x715, 0x77F)])
    self.assertTrue(any(t.subaddress is not None for t in profiles))

  def test_codes_survive_interruption_during_details(self):
    target = d.Target(0x7E0, 0x7E8)

    def interrupt(request):
      raise KeyboardInterrupt

    panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301"), b"\x22\xf1\x87": interrupt}})
    args = d.make_parser().parse_args(["--addr", "0x7e0", "--obd", "on", "--details"])
    with contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    self.assertEqual(report["status"], "partial")
    self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")
    self.assertEqual(report["ecus"][0]["queries"][-1]["outcome"], "interrupted")
    json.dumps(report)


class TestCLI(unittest.TestCase):
  def run_with_panda(self, panda):
    factory = unittest.mock.Mock(return_value=panda)
    factory.list.return_value = ["simulated"]
    with patch.dict(sys.modules, {"panda": SimpleNamespace(Panda=factory)}), \
         patch.object(d, "check_pandad"), patch.object(d, "load_known_targets", return_value={}), \
         contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
      status = d.main(["--addr", "0x7e0", "--obd", "on", "--json", "--timeout", "0.01"])
    factory.assert_called_once_with(serial=None, cli=False)
    self.assertEqual(panda.safety_modes[-1], (SAFETY.noOutput, 0))
    self.assertTrue(panda.closed)
    return status, json.loads(out.getvalue())

  def test_json_output_and_cleanup(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301")}})
    status, report = self.run_with_panda(panda)
    self.assertEqual(status, 0)
    self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")

  def test_ignition_off_returns_preflight_error_without_transmitting(self):
    panda = FakePanda({})
    panda.health_data["ignition_line"] = False
    status, report = self.run_with_panda(panda)
    self.assertEqual(status, 2)
    self.assertFalse(report["preflight"]["ready"])
    self.assertIn("Ignition not detected", report["errors"][0])
    self.assertEqual(panda.sent, [])
    with contextlib.redirect_stdout(io.StringIO()) as out:
      d.print_report(report)
    self.assertIn("No ECU scan performed", out.getvalue())

  def test_unreadable_health_returns_setup_error_and_still_cleans_up(self):
    panda = FakePanda({})
    with patch.object(panda, "health", side_effect=RuntimeError("health packet version mismatch")):
      status, report = self.run_with_panda(panda)
    self.assertEqual(status, 2)
    self.assertIn("health packet version mismatch", report["errors"][0])
    self.assertEqual(panda.sent, [])

  def test_hardware_error_and_interrupt_still_cleanup(self):
    target = d.Target(0x7E0, 0x7E8)
    for exception in (OSError("disconnected"), KeyboardInterrupt()):
      def fail(request, error=exception):
        raise error

      with self.subTest(exception=exception):
        panda = FakePanda({target: {b"\x19\x02\xff": fail}})
        status, report = self.run_with_panda(panda)
        self.assertEqual(status, 1)
        self.assertTrue(report["errors"])

  def test_running_pandad_refuses_before_hardware_import(self):
    with patch.object(d.subprocess, "run", return_value=SimpleNamespace(returncode=0)), contextlib.redirect_stdout(io.StringIO()) as out:
      code = d.main(["--json"])
    report = json.loads(out.getvalue())
    self.assertEqual(code, 2)
    self.assertTrue(report["setup_error"])
    self.assertIn("pandad is running", report["errors"][0])

  def test_invalid_numeric_options(self):
    for value in ("nan", "inf", "0", "-1"):
      with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
        d.main(["--timeout", value])

  def test_offline_dataset(self):
    data = json.loads(d.DATASET.read_text())
    self.assertEqual(len(data["labels"]), 9533)
    self.assertEqual(data["revision"], "bc58b0eb7273226a1aabae98e956b70b8362bda1")
    self.assertIn("Misfire", data["labels"]["P0301"])
    self.assertNotIn("P1234", data["labels"])


class TestPreflight(unittest.TestCase):
  def test_either_ignition_source_is_sufficient(self):
    panda = FakePanda({})
    panda.health_data.update(ignition_line=False, ignition_can=True, car_harness_status=2)
    result = d.hardware_preflight(panda)
    self.assertTrue(result["ready"])
    self.assertTrue(result["evidence"]["ignition_can"])

  def test_missing_harness_and_panda_faults_block_scan(self):
    for field, value, failed_check in (("car_harness_status", 0, "harness"), ("faults", 1, "panda_health")):
      with self.subTest(field=field):
        panda = FakePanda({})
        panda.health_data[field] = value
        args = d.make_parser().parse_args([])
        with contextlib.redirect_stderr(io.StringIO()):
          report = d.scan(panda, args, {}, {}, SAFETY)
        self.assertTrue(report["setup_error"])
        self.assertEqual(panda.sent, [])
        self.assertEqual(panda.safety_modes, [])
        check = next(c for c in report["preflight"]["checks"] if c["name"] == failed_check)
        self.assertEqual(check["status"], "fail")

  def test_external_obd_panda_does_not_require_harness_ignition_signal(self):
    panda = FakePanda({})
    panda.internal = False
    panda.health_data.update(ignition_line=False, car_harness_status=0)
    result = d.hardware_preflight(panda)
    self.assertTrue(result["ready"])
    self.assertEqual(result["checks"][0]["status"], "warning")

  def test_obd_rx_proves_activity_not_adapter_identity(self):
    engine = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({engine: {b"\x01\x00": bytes.fromhex("410000000000")}})
    check = d.check_obd_link(panda, time.monotonic() + 1, wait=0)
    self.assertEqual(check["status"], "pass")
    self.assertIsNone(check["comma_power_present"])
    self.assertGreater(check["received_frames"], 0)

  def test_missing_obd_connection_skips_route_and_keeps_harness_scan(self):
    engine = d.Target(0x7E0, 0x7E8, bus=0, obd=False)
    panda = FakePanda({engine: {b"\x03": bytes.fromhex("43010301")}})
    panda.obd_disconnected = True
    args = d.make_parser().parse_args(["--addr", "0x7e0", "--bus", "1", "--bus", "0", "--obd", "on", "--timeout", "0.01"])
    with contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    self.assertEqual(report["routes"][0]["outcome"], "skipped")
    self.assertIn("comma power", report["warnings"][0])
    self.assertIsNone(report["routes"][0]["preflight"]["comma_power_present"])
    self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")
    self.assertEqual(report["status"], "partial")
    self.assertEqual([data[1:3] for _, data, bus in panda.sent if bus == 1], [b"\x01\x00", b"\x01\x00"])

  def test_silent_bus_and_stale_errors_do_not_prove_missing_comma_power(self):
    panda = FakePanda({})
    panda.can_health_data.update(total_error_cnt=20, last_stored_error="AckError")
    check = d.check_obd_link(panda, time.monotonic() + 1, wait=0)
    self.assertEqual(check["status"], "warning")
    self.assertIsNone(check["comma_power_present"])

  def test_echoes_and_other_buses_do_not_prove_obd_connectivity(self):
    panda = FakePanda({})
    echoed = d.single_frame(b"\x01\x00")
    with patch.object(panda, "can_recv", return_value=[(0x7DF, echoed, 129), (0x7DF, echoed, 193), (0x123, b"\x01", 0)]):
      check = d.check_obd_link(panda, time.monotonic() + 1, wait=0)
    self.assertEqual(check["received_frames"], 0)
    self.assertEqual(check["status"], "warning")

  def test_expired_deadline_does_not_send_obd_probe(self):
    panda = FakePanda({})
    check = d.check_obd_link(panda, time.monotonic() - 1)
    self.assertEqual(panda.sent, [])
    self.assertEqual(check["status"], "warning")


if __name__ == "__main__":
  unittest.main()
