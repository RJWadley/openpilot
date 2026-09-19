import contextlib
import io
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.structs import CarParams
from tools.scripts.car import diagnose as d


SAFETY = CarParams.SafetyModel
MISFIRE_ENTRY = {"code": "P0301", "title": {"en": "Cylinder 1 Misfire Detected", "de": "Fehlzündung Zylinder 1"},
                 "description": {"en": "Example explanation", "de": "Beispiel"}, "common_causes": [{"label": {"en": "Example cause"}}],
                 "repair": {"estimated_cost_eur": [10, 100]}, "sources": ["https://example.com/reference"]}


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


class FakeClock:
  def __init__(self):
    self.now = 0.0

  def monotonic(self):
    return self.now

  def sleep(self, seconds):
    self.now += seconds


class TestDiscovery(unittest.TestCase):
  def discover(self, panda, probes, wait=0):
    return d.discover(panda, probes, 1, True, time.monotonic() + 2, probe_wait=wait, timeout=0.005)

  def test_reply_addresses_are_learned_without_brand_metadata(self):
    targets = {d.Target(0x710, 0x77A), d.Target(0x600, 0x650), d.Target(0x7FA, 0x680),
               d.Target(0x18DA10F1, 0x18DAF110)}
    panda = FakePanda({target: {} for target in targets})
    found, _, report = self.discover(panda, {(t.tx, t.subaddress) for t in targets})
    self.assertEqual(found, targets)
    self.assertEqual(report["unanswered"], [])
    self.assertEqual(report["unconfirmed"], [])
    self.assertEqual(report["ambiguous"], [])
    self.assertTrue(all(d.read_request(request) for _, request in panda.requests))

  def test_late_tester_reply_is_not_assigned_to_the_next_address(self):
    target = d.Target(0x710, 0x77A)
    panda = FakePanda({target: {}})
    original_send = panda.can_send

    def send(address, data, bus, timeout=0):
      original_send(address, data, bus, timeout)
      if address == 0x711 and d.frame_payload(data) == b"\x3e\x00":
        panda.queue.append((target.rx, d.single_frame(b"\x7e\x00"), bus))

    with patch.object(panda, "can_send", side_effect=send):
      found, _, report = self.discover(panda, {(0x710, None), (0x711, None)})
    self.assertEqual(found, {target})
    self.assertEqual(report["unconfirmed"][0]["tx_address"], "0x711")

  def test_timed_reply_inside_window_is_learned_outside_window_is_unconfirmed(self):
    def run(delay):
      target = d.Target(0x710, 0x77A)
      panda = FakePanda({target: {}})
      clock, delayed = FakeClock(), []
      original_send, original_recv = panda.can_send, panda.can_recv

      def send(address, data, bus, timeout=0):
        original_send(address, data, bus, timeout)
        if address == target.tx and d.frame_payload(data) == b"\x3e\x00":
          delayed.extend((clock.now + delay, msg) for msg in original_recv())

      def recv():
        ready = [msg for at, msg in delayed if at <= clock.now]
        delayed[:] = [(at, msg) for at, msg in delayed if at > clock.now]
        return original_recv() + ready

      with patch.object(d.time, "monotonic", side_effect=clock.monotonic), patch.object(d.time, "sleep", side_effect=clock.sleep), \
           patch.object(panda, "can_send", side_effect=send), patch.object(panda, "can_recv", side_effect=recv):
        found, _, report = self.discover(panda, {(0x710, None), (0x711, None)}, wait=0.01)
      return target, found, report

    for delay, expected_found in ((0.006, True), (0.016, False)):
      with self.subTest(delay=delay):
        target, found, report = run(delay)
        self.assertEqual(found, {target} if expected_found else set())
        if not expected_found:
          self.assertEqual(report["unconfirmed"][0]["tx_address"], "0x711")

  def test_multiple_confirmed_replies_are_reported_as_ambiguous(self):
    targets = {d.Target(0x710, 0x77A), d.Target(0x710, 0x77B)}
    panda = FakePanda({t: {} for t in targets})
    found, _, report = self.discover(panda, {(0x710, None)})
    self.assertEqual(found, set())
    self.assertEqual({item["rx_address"] for item in report["ambiguous"]}, {"0x77a", "0x77b"})

  def test_shared_reply_address_is_not_counted_as_two_modules(self):
    targets = {d.Target(0x710, 0x77A), d.Target(0x711, 0x77A)}
    panda = FakePanda({t: {} for t in targets})
    found, _, report = self.discover(panda, {(0x710, None), (0x711, None)})
    self.assertEqual(found, set())
    self.assertEqual({item["tx_address"] for item in report["ambiguous"]}, {"0x710", "0x711"})

  def test_confirmation_handles_multiframe_identification(self):
    target = d.Target(0x710, 0x77A)
    identity = b"\x62\xf1\x97Example module"
    panda = FakePanda({target: {b"\x22\xf1\x97": identity}})
    found, _, report = self.discover(panda, {(0x710, None)})
    self.assertEqual(found, {target})
    self.assertTrue(any(data[0] == 0x30 for _, data, _ in panda.sent))
    self.assertTrue(any(identity.hex() in item.get("responses", []) for item in report["responses"]))

  def test_silent_identification_falls_back_to_a_dtc_read(self):
    target = d.Target(0x710, 0x77A)
    panda = FakePanda({target: {b"\x22\xf1\x97": None, b"\x19\x02\xff": b"\x59\x02\xff"}})
    found, _, _ = self.discover(panda, {(0x710, None)})
    self.assertEqual(found, {target})

  def test_emissions_endpoint_without_tester_present_is_still_discovered(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x01\x00": bytes.fromhex("410000000000"), b"\x3e\x00": None}})
    found, emissions, report = self.discover(panda, {(target.tx, None)})
    self.assertEqual(found, {target})
    self.assertEqual(emissions, {target})
    self.assertEqual(report["unanswered"], [])
    self.assertTrue(any(item.get("name") == "obd_confirmation" and item["outcome"] == "ok" for item in report["responses"]))

  def test_stale_echo_and_wrong_bus_frames_do_not_discover_modules(self):
    panda = FakePanda({})
    reply = d.single_frame(b"\x7e\x00")
    panda.queue.append((0x77A, reply, 1))
    original_send = panda.can_send

    def send(address, data, bus, timeout=0):
      original_send(address, data, bus, timeout)
      panda.queue.extend([(0x77A, reply, 0), (0x77A, reply, 129), (0x77A, reply, 193),
                          (address, data, 1), (0x77A, d.single_frame(b"\x7f\x22\x11"), 1)])

    with patch.object(panda, "can_send", side_effect=send):
      found, _, report = self.discover(panda, {(0x710, None)})
    self.assertEqual(found, set())
    self.assertEqual(report["unconfirmed"], [])

  def test_deadline_prevents_probes_and_reports_remaining_candidates(self):
    panda = FakePanda({})
    found, _, report = d.discover(panda, {(0x710, None)}, 1, True, time.monotonic() - 1)
    self.assertFalse(found)
    self.assertEqual(panda.sent, [])
    self.assertEqual(report["not_probed"], 1)

  def test_deadline_stops_during_a_probe_without_starting_confirmation(self):
    target = d.Target(0x710, 0x77A)
    panda, clock = FakePanda({target: {}}), FakeClock()
    with patch.object(d.time, "monotonic", side_effect=clock.monotonic), patch.object(d.time, "sleep", side_effect=clock.sleep):
      found, _, report = d.discover(panda, {(0x710, None), (0x711, None)}, 1, True, 0.15, probe_wait=0.1)
    self.assertEqual(found, set())
    self.assertEqual(report["not_probed"], 1)
    self.assertEqual(report["unconfirmed"][0]["tx_address"], "0x710")
    self.assertEqual([address for address, _, _ in panda.sent], [0x7DF, 0x18DB33F1, 0x710])
    self.assertAlmostEqual(clock.now, 0.15)

  def test_generic_probe_ranges_respect_panda_safety(self):
    probes = d.generic_probes()
    self.assertTrue({(0x24B, None), (0x600, None), (0x700, None), (0x7FF, None), (0x18DA10F1, None)} <= probes)
    self.assertTrue(all(d.valid_tx(tx) for tx, _ in probes))
    self.assertNotIn((0x7DF, None), probes)


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

  def test_uds_filters_only_non_fault_statuses(self):
    for status in range(256):
      with self.subTest(status=status):
        records = d.parse_uds_codes(b"\xff\x90\x16\x14" + bytes([status]), None)
        self.assertEqual(bool(records), bool(status & 0xAF))
        if records:
          self.assertEqual(records[0]["status_byte"], status)

  def test_uds_filter_respects_supported_bits_and_retains_history(self):
    self.assertEqual(d.parse_uds_codes(bytes.fromhex("50 901614 ff"), None), [])
    for status in (0x01, 0x02, 0x04, 0x08, 0x20, 0x80):
      with self.subTest(status=status):
        self.assertEqual(len(d.parse_uds_codes(b"\xff\x90\x16\x14" + bytes([status]), None)), 1)

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
    for raw, outcome in (("7f0311", "unsupported"), ("7f0333", "rejected"), ("7f03", "malformed"), ("7f031100", "malformed")):
      with self.subTest(raw=raw):
        panda = FakePanda({target: {b"\x03": bytes.fromhex(raw)}})
        result = d.Reader(transport(panda), target).read("stored", b"\x03", b"\x43")
        self.assertEqual(result["outcome"], outcome)
        self.assertEqual(result["responses"], [raw])

  def test_late_replies_from_other_services_are_ignored_but_retained(self):
    target = d.Target(0x710, 0x77A)
    for stale in (b"\x7e\x00", b"\x7f\x3e\x12"):
      with self.subTest(stale=stale):
        reply = b"\x62\xf1\x97ECU"
        panda = FakePanda({target: {b"\x22\xf1\x97": [stale, reply]}})
        result = d.Reader(transport(panda), target).read("identity", b"\x22\xf1\x97", b"\x62\xf1\x97")
        self.assertEqual(result["outcome"], "ok")
        self.assertEqual(result["responses"], [stale.hex(), reply.hex()])

  def test_write_requests_rejected_before_transmission(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({})
    wire = transport(panda)
    for request in (b"\x04", b"\x14\xff\xff\xff", b"\x27\x01", b"\x2e\xf1\x90", b"\x31\x01", b"\x10\x02"):
      with self.subTest(request=request), self.assertRaises(ValueError):
        wire.exchange(target, request)
    self.assertEqual(panda.sent, [])


class IsolatedCacheTest(unittest.TestCase):
  def setUp(self):
    directory = tempfile.TemporaryDirectory()
    self.addCleanup(directory.cleanup)
    self.cache_path = Path(directory.name) / "modules.json"
    self.enterContext(patch.object(d, "module_cache_path", return_value=self.cache_path))


class TestScanning(IsolatedCacheTest):
  def test_auto_scan_twenty_nonstandard_pairs_without_brand_metadata(self):
    targets = {d.Target(0x700 + i, 0x76A + i, obd=False) for i in range(20)}
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff80011389")} for target in targets})
    args = d.make_parser().parse_args(["--bus", "1", "--obd", "off"])
    clock = FakeClock()
    with patch.object(d.time, "monotonic", side_effect=clock.monotonic), patch.object(d.time, "sleep", side_effect=clock.sleep), \
         contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    self.assertEqual(len(report["ecus"]), 20)
    self.assertTrue(all(ecu["dtc_read"] for ecu in report["ecus"]))
    self.assertTrue(all(not ecu["identity_candidates"] for ecu in report["ecus"]))
    self.assertEqual(report["discovery"][0]["not_probed"], 0)
    self.assertFalse(report["vehicle_coverage_complete"])
    self.assertFalse(report["deadline_reached"])
    self.assertEqual(panda.safety_modes, [(SAFETY.elm327, 1)])
    self.assertTrue(all(d.read_request(d.frame_payload(data)) for _, data, _ in panda.sent))
    json.dumps(report)

  def test_discovery_all_functional_responders_and_bus_filter(self):
    engine, transmission = d.Target(0x7E0, 0x7E8), d.Target(0x7E1, 0x7E9)
    extended = d.Target(0x18DA10F1, 0x18DAF110)
    vw = d.Target(0x715, 0x77F)
    sub = d.Target(0x750, 0x758, subaddress=0x0F)
    wrong_bus = d.Target(0x7E2, 0x7EA, bus=0, obd=False)
    mode01 = {b"\x01\x00": bytes.fromhex("4100be3fa813")}
    panda = FakePanda({engine: mode01, transmission: mode01, extended: mode01, vw: {}, sub: {}, wrong_bus: mode01})
    panda.queue.append((wrong_bus.rx, d.single_frame(bytes.fromhex("4100be3fa813")), 0))
    probes = {(t.tx, t.subaddress) for t in (engine, transmission, extended, vw, sub)}
    found, emissions, report = d.discover(panda, probes, 1, True, time.monotonic() + 1, probe_wait=0)
    self.assertEqual(found, {engine, transmission, extended, vw, sub})
    self.assertEqual(emissions, {engine, transmission, extended})
    self.assertEqual(report["unanswered"], [])

  def test_scan_keeps_engine_when_airbag_cannot_read(self):
    engine, airbag = d.Target(0x7E0, 0x7E8), d.Target(0x715, 0x77F)
    panda = FakePanda({engine: {b"\x03": bytes.fromhex("43010301"), b"\x01\x00": bytes.fromhex("410000000000")}, airbag: {}})
    args = d.make_parser().parse_args(["--bus", "1", "--obd", "on", "--timeout", "0.02"])
    with patch.object(d, "generic_probes", return_value={(engine.tx, None), (airbag.tx, None)}), contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {"entries": {"P0301": MISFIRE_ENTRY}}, {}, SAFETY)
    self.assertEqual(report["status"], "partial")
    self.assertFalse(report["vehicle_coverage_complete"])
    results = {ecu["tx_address"]: ecu for ecu in report["ecus"]}
    self.assertFalse(results["0x715"]["dtc_read"])
    code = results["0x7e0"]["codes"][0]
    self.assertEqual(code["code"], "P0301")
    self.assertIn("lookup", code)
    self.assertEqual(code["lookup"]["entry"], MISFIRE_ENTRY)
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
    result = d.query_ecu(transport(panda), target, {"B0001": {"title": {"en": "Example"}}})
    self.assertEqual(result["codes"][0]["code"], "0x800113")
    self.assertNotIn("lookup", result["codes"][0])

  def test_unmapped_airbag_gets_lossless_decimal_and_real_ecu_context(self):
    target = d.Target(0x715, 0x77F)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff9016148992250110"),
                                b"\x22\xf1\x87": b"\x62\xf1\x875Q0959655J ",
                                b"\x22\xf1\x97": b"\x62\xf1\x97AirbagVW20   "}})
    result = d.query_ecu(transport(panda), target, {})
    self.assertEqual(len(result["codes"]), 1)
    code = result["codes"][0]
    self.assertEqual(code["code"], "0x901614")
    self.assertEqual(code["display_code"], "9442836 (0x901614)")
    self.assertEqual(code["search"], {"codes": ["9442836", "0x901614"], "query": "9442836 AirbagVW20 5Q0959655J"})
    self.assertNotIn("B1016", json.dumps(code))
    self.assertEqual(result["ignored_non_fault_records"], 1)
    self.assertEqual(result["identity"]["part_number"], "5Q0959655J")
    self.assertFalse(any(request[:2] in (b"\x19\x04", b"\x19\x06") for _, request in panda.requests))

  def test_verified_format_keeps_searchable_standard_code_and_failure_type(self):
    target = d.Target(0x715, 0x77F)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff90161489"),
                                b"\x19\x01\xff": bytes.fromhex("5901ff040001")}})
    code = d.query_ecu(transport(panda), target, {})["codes"][0]
    self.assertEqual(code["code"], "B1016")
    self.assertEqual(code["display_code"], "B1016-14")
    self.assertEqual(code["search"]["codes"], ["B1016 14", "B101614", "B1016", "9442836", "0x901614"])

  def test_obdex_full_entry_is_separate_from_vehicle_status_and_not_duplicated_in_query(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301")}})
    result = d.query_ecu(transport(panda), target, {"P0301": MISFIRE_ENTRY}, obd=True)
    code = result["codes"][0]
    self.assertEqual(code["lookup"]["entry"], MISFIRE_ENTRY)
    self.assertEqual(code["status"], ["stored"])
    self.assertNotIn("search", code)
    query = next(q for q in result["queries"] if q["name"] == "obd_stored")
    self.assertNotIn("lookup", query["data"][0])
    with contextlib.redirect_stdout(io.StringIO()) as out:
      d.print_report({"status": "partial", "ecus": [result], "errors": []})
    for text in ("Example explanation", "Example cause", "estimated_cost_eur", "https://example.com/reference", "Fehlzündung"):
      self.assertIn(text, out.getvalue())

  def test_non_faults_are_ignored_before_applying_detail_limit(self):
    target = d.Target(0x715, 0x77F)
    records = b"".join(i.to_bytes(3, "big") + b"\x10" for i in range(20)) + bytes.fromhex("90161489")
    panda = FakePanda({target: {b"\x19\x02\xff": b"\x59\x02\xff" + records}})
    result = d.query_ecu(transport(panda), target, {}, details=True, max_details=1)
    self.assertEqual(result["ignored_non_fault_records"], 20)
    self.assertEqual(result["details_omitted"], 0)
    self.assertEqual([c["code"] for c in result["codes"]], ["0x901614"])
    requests = [request for _, request in panda.requests if request[:2] in (b"\x19\x04", b"\x19\x06")]
    self.assertEqual(requests, [bytes.fromhex("1904901614ff"), bytes.fromhex("1906901614ff")])
    query = next(q for q in result["queries"] if q["name"] == "uds_codes")
    self.assertEqual(len(query["data"]), 1)
    self.assertEqual(query["responses"], [(b"\x59\x02\xff" + records).hex()])

  def test_only_non_fault_records_produce_no_fault_details(self):
    target = d.Target(0x715, 0x77F)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff92250110902501409025125090251300")}})
    result = d.query_ecu(transport(panda), target, {}, details=True)
    self.assertTrue(result["dtc_read"])
    self.assertEqual(result["codes"], [])
    self.assertEqual(result["ignored_non_fault_records"], 4)
    self.assertFalse(any(request[:2] in (b"\x19\x04", b"\x19\x06") for _, request in panda.requests))

  def test_unknown_engine_code_is_not_guessed_from_an_obd_match(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff003c5e2f"), b"\x03": bytes.fromhex("43010202")}})
    entry = {"code": "P0202", "title": {"en": "Injector Circuit Malfunction — Cylinder 2"}}
    result = d.query_ecu(transport(panda), target, {"P0202": entry}, obd=True)
    unknown, matched = result["codes"]
    self.assertEqual(unknown["display_code"], "15454 (0x003C5E)")
    self.assertEqual(unknown["search"]["codes"], ["15454", "0x003C5E"])
    self.assertNotIn("lookup", unknown)
    self.assertEqual(matched["lookup"]["entry"], entry)

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

  def test_targeting_learns_an_unknown_reply_offset_without_brand_metadata(self):
    target = d.Target(0x710, 0x77A, obd=False)
    panda = FakePanda({target: {b"\x19\x02\xff": bytes.fromhex("5902ff80011389")}})
    args = d.make_parser().parse_args(["--addr", "0x710", "--obd", "off", "--probe-timeout", "0.001", "--timeout", "0.01"])
    with contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    self.assertEqual(report["ecus"][0]["rx_address"], "0x77a")
    self.assertTrue(report["ecus"][0]["dtc_read"])
    self.assertEqual(report["ecus"][0]["identity_candidates"], [])

  def test_subaddress_hints_are_optional_and_explicit_subaddress_is_respected(self):
    hint = d.Target(0x750, 0x758, subaddress=0x0F)
    args = d.make_parser().parse_args(["--addr", "0x750"])
    self.assertEqual(d.selected_probes(args, {hint: []}, 1, True), {(0x750, None), (0x750, 0x0F)})
    args.subaddress = 2
    self.assertEqual(d.selected_probes(args, {hint: []}, 1, True), {(0x750, 2)})

  def test_explicit_rx_bypasses_discovery(self):
    target = d.Target(0x710, 0x77A, obd=False)
    panda = FakePanda({target: {b"\x19\x02\xff": b"\x59\x02\xff"}})
    args = d.make_parser().parse_args(["--addr", "0x710", "--rx-addr", "0x77a", "--obd", "off"])
    with patch.object(d, "discover", side_effect=AssertionError("discovery must be bypassed")), contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    self.assertEqual(report["status"], "partial")
    self.assertEqual(report["discovery"], [])

  def test_no_response_is_failure_not_no_faults(self):
    args = d.make_parser().parse_args(["--addr", "0x7e0", "--obd", "on", "--timeout", "0.001"])
    with contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(FakePanda({}), args, {}, {}, SAFETY)
    self.assertEqual(report["status"], "failed")
    self.assertEqual(report["ecus"], [])
    self.assertEqual(report["discovery"][0]["unanswered"], [{"tx_address": "0x7e0", "subaddress": None}])

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


class TestRoutesAndCache(IsolatedCacheTest):
  engine = d.Target(0x7E0, 0x7E8)
  airbag = d.Target(0x715, 0x77F)
  vin = b"WVWZZZAUZGW000001"

  def ecus(self, vin=None):
    return {self.engine: {b"\x22\xf1\x90": b"\x62\xf1\x90" + (vin or self.vin), b"\x19\x02\xff": b"\x59\x02\xff",
                          b"\x01\x00": bytes.fromhex("410000000000")},
            self.airbag: {b"\x19\x02\xff": bytes.fromhex("5902ff90161488")}}

  def run_scan(self, argv=(), ecus=None, panda=None):
    panda = panda or FakePanda(self.ecus() if ecus is None else ecus)
    args = d.make_parser().parse_args(list(argv))
    clock = FakeClock()
    with patch.object(d, "generic_probes", return_value={(self.engine.tx, None), (self.airbag.tx, None)}), \
         patch.object(d.time, "monotonic", side_effect=clock.monotonic), patch.object(d.time, "sleep", side_effect=clock.sleep), \
         contextlib.redirect_stderr(io.StringIO()):
      report = d.scan(panda, args, {}, {}, SAFETY)
    return report, panda

  def test_route_selection_and_explicit_overrides(self):
    for argv, routes in (
      ([], [(1, True)]),
      (["--fast"], [(1, True)]),
      (["--broad"], [(1, True), (1, False), (0, False), (2, False)]),
      (["--addr", "0x715"], [(1, True)]),
      (["--obd", "auto"], [(1, True), (1, False)]),
      (["--broad", "--bus", "0"], [(0, False)]),
      (["--broad", "--obd", "on"], [(1, True), (0, False), (2, False)]),
      (["--bus", "2", "--bus", "0", "--bus", "2"], [(2, False), (0, False)]),
    ):
      with self.subTest(argv=argv):
        self.assertEqual(d.selected_routes(d.make_parser().parse_args(argv)), routes)

  def test_default_scan_saves_only_obd_addresses_not_faults_or_raw_vin(self):
    report, panda = self.run_scan()
    self.assertEqual(panda.safety_modes, [(SAFETY.elm327, 0)])
    self.assertTrue(report["cache"]["updated"])
    cache = d.load_module_cache(self.cache_path)
    self.assertEqual(len(cache["routes"]), 1)
    self.assertEqual({d.cached_target(t) for t in cache["routes"][0]["targets"]}, {self.engine, self.airbag})
    contents = self.cache_path.read_text()
    self.assertNotIn("901614", contents)
    self.assertNotIn(self.vin.decode(), contents)
    self.assertFalse(report["vehicle_coverage_complete"])

  def test_fast_reads_fresh_faults_without_discovery(self):
    self.run_scan()
    ecus = self.ecus()
    ecus[self.airbag][b"\x19\x02\xff"] = bytes.fromhex("5902ff92250120")
    with patch.object(d, "discover", side_effect=AssertionError("Must reuse cached targets")):
      report, panda = self.run_scan(["--fast"], ecus)
    self.assertEqual(report["cache"]["routes_reused"], 1)
    self.assertEqual(report["discovery"], [])
    self.assertEqual(report["ecus"][0]["codes"][0]["raw_dtc"], "922501")
    self.assertFalse(any(request == b"\x3e\x00" for _, request in panda.requests))
    self.assertIn((self.engine, b"\x22\xf1\x90"), panda.requests)
    self.assertFalse(report["vehicle_coverage_complete"])

  def test_broad_cache_includes_empty_routes_but_fast_alone_stays_obd_only(self):
    ecus = self.ecus()
    ecus[d.Target(0x715, 0x77F, bus=0, obd=False)] = ecus[self.airbag]
    self.run_scan(["--broad"], ecus)
    with patch.object(d, "discover", side_effect=AssertionError("All routes cached")):
      narrow, panda = self.run_scan(["--fast"], ecus)
      broad, _ = self.run_scan(["--fast", "--broad"], ecus)
    self.assertEqual(panda.safety_modes, [(SAFETY.elm327, 0)])
    self.assertEqual(len(narrow["ecus"]), 2)
    self.assertEqual(len(broad["ecus"]), 3)
    self.assertEqual(broad["cache"]["routes_reused"], 4)

  def test_fast_broad_discovers_routes_missing_from_obd_cache(self):
    self.run_scan()
    report, _ = self.run_scan(["--fast", "--broad"])
    self.assertEqual(report["cache"]["routes_reused"], 1)
    self.assertEqual(len(report["discovery"]), 3)
    self.assertEqual(len(d.load_module_cache(self.cache_path)["routes"]), 4)

  def test_narrow_refresh_preserves_previously_cached_harness_routes(self):
    self.run_scan(["--broad"])
    report, _ = self.run_scan()
    self.assertTrue(report["cache"]["updated"])
    self.assertEqual(len(d.load_module_cache(self.cache_path)["routes"]), 4)

  def test_transient_vin_failure_preserves_other_routes_after_same_vehicle_is_verified(self):
    self.run_scan(["--broad"])
    ecus = self.ecus()
    replies = iter([None, b"\x62\xf1\x90" + self.vin])
    ecus[self.engine][b"\x22\xf1\x90"] = lambda request: next(replies)
    report, _ = self.run_scan(["--fast"], ecus)
    self.assertEqual(report["cache"]["routes_reused"], 0)
    self.assertTrue(report["cache"]["updated"])
    self.assertEqual(len(d.load_module_cache(self.cache_path)["routes"]), 4)

  def test_missing_cache_falls_back_and_populates_cache(self):
    report, _ = self.run_scan(["--fast"])
    self.assertEqual(report["cache"]["routes_reused"], 0)
    self.assertTrue(report["cache"]["updated"])
    self.assertEqual(len(report["ecus"]), 2)
    self.assertTrue(any("No module cache" in w for w in report["warnings"]))

  def test_vehicle_change_discovers_and_discards_other_vehicle_routes(self):
    self.run_scan(["--broad"])
    previous = d.load_module_cache(self.cache_path)["identity"]["vin_hash"]
    report, _ = self.run_scan(["--fast"], self.ecus(b"WVWZZZAUZGW000002"))
    cache = d.load_module_cache(self.cache_path)
    self.assertEqual(report["cache"]["routes_reused"], 0)
    self.assertEqual(len(report["discovery"]), 1)
    self.assertNotEqual(cache["identity"]["vin_hash"], previous)
    self.assertEqual(len(cache["routes"]), 1)

  def test_unreadable_or_invalid_vin_falls_back_without_overwriting_cache(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    for vin in (None, b"", b"0" * 17, b"I" * 17, b"\xff" * 17):
      with self.subTest(vin=vin):
        ecus = self.ecus()
        ecus[self.engine][b"\x22\xf1\x90"] = None if vin is None else b"\x62\xf1\x90" + vin
        report, _ = self.run_scan(["--fast"], ecus)
        self.assertEqual(report["cache"]["routes_reused"], 0)
        self.assertEqual(len(report["discovery"]), 1)
        self.assertEqual(self.cache_path.read_bytes(), previous)

  def test_obd_vin_fallback_is_cached_and_verified(self):
    ecus = self.ecus()
    del ecus[self.engine][b"\x22\xf1\x90"]
    ecus[self.engine][b"\x09\x02"] = b"\x49\x02\x01" + self.vin
    self.run_scan(ecus=ecus)
    self.assertEqual(d.load_module_cache(self.cache_path)["identity"]["request"], "0902")
    report, panda = self.run_scan(["--fast"], ecus)
    self.assertEqual(report["cache"]["routes_reused"], 1)
    self.assertTrue(all(d.read_request(request) for _, request in panda.requests))

  def test_corrupt_cache_falls_back_without_breaking_diagnostics(self):
    for data in ("not json", "null", "[]", '{"version":99}', '{"version":1,"routes":[]}'):
      with self.subTest(data=data):
        self.cache_path.write_text(data)
        report, _ = self.run_scan(["--fast"])
        self.assertEqual(report["status"], "partial")
        self.assertTrue(report["cache"]["updated"])
        self.assertEqual(report["cache"]["routes_reused"], 0)

  def test_unsafe_or_ambiguous_cached_addresses_are_not_used(self):
    self.run_scan()
    good = self.cache_path.read_text()
    for change in ("request", "address", "bus", "subaddress", "duplicate"):
      with self.subTest(change=change):
        cache = json.loads(good)
        item = cache["routes"][0]["targets"][0]
        if change == "request":
          cache["identity"]["request"] = "2701"
        elif change == "address":
          item["tx_address"] = "0x123"
        elif change == "bus":
          item["bus"] = 2
        elif change == "subaddress":
          item["subaddress"] = 256
        else:
          cache["routes"][0]["targets"].append(dict(item, rx_address="0x780"))
        self.cache_path.write_text(json.dumps(cache))
        report, panda = self.run_scan(["--fast"])
        self.assertEqual(report["cache"]["routes_reused"], 0)
        self.assertTrue(all(d.read_request(request) for _, request in panda.requests))
        self.assertFalse(any(tx == 0x123 for tx, _, _ in panda.sent))

  def test_targeted_or_explicit_scan_preserves_full_cache(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    for argv in (["--addr", "0x715"], ["--fast", "--addr", "0x715"], ["--fast", "--addr", "0x715", "--rx-addr", "0x77f"]):
      with self.subTest(argv=argv):
        report, _ = self.run_scan(argv)
        self.assertEqual(len(report["ecus"]), 1)
        self.assertEqual(report["ecus"][0]["tx_address"], "0x715")
        self.assertEqual(self.cache_path.read_bytes(), previous)

  def test_fast_preserves_emissions_and_zero_subaddress(self):
    extended = d.Target(0x18DA10F1, 0x18DAF110)
    sub = d.Target(0x750, 0x758, subaddress=0)
    ecus = self.ecus()
    ecus[extended] = {b"\x01\x00": bytes.fromhex("410000000000"), b"\x03": bytes.fromhex("43010301")}
    ecus[sub] = {b"\x19\x02\xff": b"\x59\x02\xff"}
    with patch.object(d, "selected_probes", return_value={(t.tx, t.subaddress) for t in ecus}):
      self.run_scan(ecus=ecus)
    with patch.object(d, "discover", side_effect=AssertionError("Must reuse cached targets")):
      report, panda = self.run_scan(["--fast"], ecus)
    self.assertIn((extended, b"\x03"), panda.requests)
    self.assertIn((sub, b"\x19\x02\xff"), panda.requests)
    self.assertTrue(any(ecu["subaddress"] == 0 for ecu in report["ecus"]))
    targeted, _ = self.run_scan(["--fast", "--addr", "0x750", "--subaddress", "0"], ecus)
    self.assertEqual(len(targeted["ecus"]), 1)
    self.assertEqual(targeted["ecus"][0]["subaddress"], 0)

  def test_unconfirmed_discovery_does_not_replace_cache(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    ecus = self.ecus()
    ecus[self.airbag] = {b"\x22\xf1\x97": None, b"\x19\x02\xff": None}
    report, _ = self.run_scan(ecus=ecus)
    self.assertTrue(report["discovery"][0]["unconfirmed"])
    self.assertFalse(report["cache"]["updated"])
    self.assertEqual(self.cache_path.read_bytes(), previous)

  def test_fast_keeps_ignition_and_obd_preflight(self):
    self.run_scan()
    panda = FakePanda(self.ecus())
    panda.health_data["ignition_line"] = False
    report, _ = self.run_scan(["--fast"], panda=panda)
    self.assertTrue(report["setup_error"])
    self.assertFalse(panda.sent)
    panda = FakePanda(self.ecus())
    panda.obd_disconnected = True
    report, _ = self.run_scan(["--fast"], panda=panda)
    self.assertEqual(report["routes"][0]["outcome"], "skipped")
    self.assertEqual(report["cache"]["routes_reused"], 0)
    self.assertFalse(report["ecus"])

  def test_silent_cached_module_remains_unavailable_not_clean(self):
    self.run_scan()
    ecus = self.ecus()
    del ecus[self.airbag]
    report, _ = self.run_scan(["--fast"], ecus)
    self.assertEqual(report["ecus"][0]["tx_address"], "0x715")
    self.assertFalse(report["ecus"][0]["dtc_read"])
    self.assertEqual(report["ecus"][0]["queries"][0]["outcome"], "timeout")
    self.assertFalse(report["cache"]["updated"])

  def test_deadline_interrupt_or_disconnection_preserves_cache(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    for failure in ("deadline", "interrupt", "disconnected"):
      with self.subTest(failure=failure):
        panda = FakePanda(self.ecus())
        argv = []
        if failure == "deadline":
          argv = ["--scan-timeout", "0.35"]
        elif failure == "interrupt":
          def interrupt(request):
            raise KeyboardInterrupt
          panda.ecus[self.airbag][b"\x19\x02\xff"] = interrupt
        else:
          panda.obd_disconnected = True
        report, _ = self.run_scan(argv, panda=panda)
        self.assertFalse(report["cache"]["updated"])
        self.assertEqual(self.cache_path.read_bytes(), previous)

  def test_cache_write_failure_is_warning_not_scan_failure(self):
    with patch.object(d, "save_module_cache", side_effect=OSError("disk full")):
      report, _ = self.run_scan()
    self.assertEqual(report["status"], "partial")
    self.assertFalse(report["errors"])
    self.assertTrue(any("disk full" in w for w in report["warnings"]))

  def test_interrupt_during_cache_save_retains_faults_and_previous_cache(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    with patch.object(d, "save_module_cache", side_effect=KeyboardInterrupt):
      report, _ = self.run_scan()
    self.assertEqual(report["status"], "partial")
    self.assertEqual(report["ecus"][0]["codes"][0]["raw_dtc"], "901614")
    self.assertEqual(self.cache_path.read_bytes(), previous)
    self.assertFalse(report["cache"]["updated"])

  def test_fast_cli_json_and_cleanup_even_on_transport_failure_or_interrupt(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    for failure in (None, OSError("disconnected"), KeyboardInterrupt()):
      with self.subTest(failure=failure):
        panda = FakePanda(self.ecus())
        if failure:
          def fail(request, error=failure):
            raise error
          panda.ecus[self.airbag][b"\x19\x02\xff"] = fail
        factory = unittest.mock.Mock(return_value=panda)
        factory.list.return_value = ["simulated"]
        clock = FakeClock()
        with patch.dict(sys.modules, {"panda": SimpleNamespace(Panda=factory)}), \
             patch.object(d, "check_pandad"), patch.object(d, "load_known_targets", return_value={}), \
             patch.object(d.time, "monotonic", side_effect=clock.monotonic), patch.object(d.time, "sleep", side_effect=clock.sleep), \
             contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
          status = d.main(["--fast", "--json"])
        report = json.loads(out.getvalue())
        self.assertEqual(report["cache"]["routes_reused"], 1)
        self.assertEqual(report["discovery"], [])
        self.assertEqual(status, 1 if failure else 0)
        self.assertEqual(panda.safety_modes[-1], (SAFETY.noOutput, 0))
        self.assertTrue(panda.closed)
        self.assertEqual(self.cache_path.read_bytes(), previous)

  def test_failed_atomic_replace_preserves_original_and_cleans_temporary_file(self):
    self.run_scan()
    previous = self.cache_path.read_bytes()
    with patch.object(Path, "replace", side_effect=OSError("disk error")):
      report, _ = self.run_scan()
    self.assertFalse(report["cache"]["updated"])
    self.assertEqual(self.cache_path.read_bytes(), previous)
    self.assertEqual(list(self.cache_path.parent.iterdir()), [self.cache_path])


class TestCLI(IsolatedCacheTest):
  def run_with_panda(self, panda):
    factory = unittest.mock.Mock(return_value=panda)
    factory.list.return_value = ["simulated"]
    with patch.dict(sys.modules, {"panda": SimpleNamespace(Panda=factory)}), \
         patch.object(d, "check_pandad"), patch.object(d, "load_known_targets", return_value={}), \
         contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()):
      status = d.main(["--addr", "0x7e0", "--obd", "on", "--json", "--timeout", "0.01", "--probe-timeout", "0.001"])
    factory.assert_called_once_with(serial=None, cli=False)
    self.assertEqual(panda.safety_modes[-1], (SAFETY.noOutput, 0))
    self.assertTrue(panda.closed)
    report = json.loads(out.getvalue())
    self.assertEqual(report["schema_version"], 3)
    return status, report

  def test_json_output_and_cleanup(self):
    target = d.Target(0x7E0, 0x7E8)
    panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301")}})
    status, report = self.run_with_panda(panda)
    self.assertEqual(status, 0)
    self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")

  def test_reported_panda_faults_warn_but_do_not_block_diagnostics(self):
    target = d.Target(0x7E0, 0x7E8)
    for faults in (8, 1, 9, 1 << 31):
      with self.subTest(faults=faults):
        panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301")}})
        panda.health_data.update(faults=faults, fault_status=1)
        status, report = self.run_with_panda(panda)
        self.assertEqual(status, 0)
        self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")
        self.assertTrue(report["preflight"]["ready"])
        self.assertEqual(report["preflight"]["evidence"]["faults"], faults)
        check = next(c for c in report["preflight"]["checks"] if c["name"] == "panda_health")
        self.assertEqual(check["status"], "warning")
        self.assertIn(check["message"], report["warnings"])
        self.assertFalse(report["errors"])
        self.assertIn((SAFETY.elm327, 0), panda.safety_modes)

  def test_panda_fault_warning_does_not_bypass_obd_connection_failure(self):
    panda = FakePanda({})
    panda.health_data["faults"] = 8
    panda.obd_disconnected = True
    status, report = self.run_with_panda(panda)
    self.assertEqual(status, 1)
    self.assertEqual(report["routes"][0]["outcome"], "skipped")
    self.assertEqual(report["routes"][0]["preflight"]["status"], "fail")
    self.assertEqual(report["ecus"], [])

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
    for option in ("--timeout", "--probe-timeout", "--scan-timeout"):
      for value in ("nan", "inf", "0", "-1"):
        with self.subTest(option=option, value=value), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
          d.main([option, value])

  def test_offline_dataset(self):
    data = d.load_dataset()
    self.assertEqual(len(data["entries"]), 9533)
    self.assertEqual(data["revision"], "bc58b0eb7273226a1aabae98e956b70b8362bda1")
    self.assertIn("Misfire", data["entries"]["P0301"]["title"]["en"])
    self.assertNotIn("P1234", data["entries"])
    for field in ("description", "affected_components", "common_causes", "repair", "flags", "related_codes", "sources", "references"):
      self.assertIn(field, data["entries"]["P0202"])
    self.assertNotIn("symptoms", data["entries"]["P0202"])  # Absent upstream at the pinned revision; do not invent fields.

  def test_corrupt_or_truncated_obdex_does_not_block_fault_reads(self):
    path = self.cache_path.parent / "broken-obdex.json.gz"
    for contents in (b"not gzip", d.gzip.compress(b"{}")[:-8], d.gzip.compress(b"not json")):
      with self.subTest(contents=contents):
        path.write_bytes(contents)
        target = d.Target(0x7E0, 0x7E8)
        panda = FakePanda({target: {b"\x03": bytes.fromhex("43010301")}})
        with patch.object(d, "DATASET", path):
          status, report = self.run_with_panda(panda)
        self.assertEqual(status, 0)
        self.assertEqual(report["ecus"][0]["codes"][0]["code"], "P0301")
        self.assertNotIn("lookup", report["ecus"][0]["codes"][0])
        self.assertTrue(any("Offline descriptions unavailable" in warning for warning in report["warnings"]))


class TestPreflight(IsolatedCacheTest):
  def test_either_ignition_source_is_sufficient(self):
    panda = FakePanda({})
    panda.health_data.update(ignition_line=False, ignition_can=True, car_harness_status=2)
    result = d.hardware_preflight(panda)
    self.assertTrue(result["ready"])
    self.assertTrue(result["evidence"]["ignition_can"])

  def test_missing_harness_blocks_scan_even_with_panda_fault_warning(self):
    for faults in (0, 8):
      with self.subTest(faults=faults):
        panda = FakePanda({})
        panda.health_data.update(car_harness_status=0, faults=faults)
        args = d.make_parser().parse_args([])
        with contextlib.redirect_stderr(io.StringIO()):
          report = d.scan(panda, args, {}, {}, SAFETY)
        self.assertTrue(report["setup_error"])
        self.assertEqual(panda.sent, [])
        self.assertEqual(panda.safety_modes, [])
        check = next(c for c in report["preflight"]["checks"] if c["name"] == "harness")
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
