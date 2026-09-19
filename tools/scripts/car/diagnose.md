# Diagnostic reader

Run on an installed comma with the vehicle parked, ignition on, and openpilot
stopped. This script takes direct ownership of the Panda. It refuses to run while
`pandad` is running and does not stop or restart openpilot for you.

From the openpilot checkout on the comma:

```sh
python tools/scripts/car/diagnose.py
python tools/scripts/car/diagnose.py --details
python tools/scripts/car/diagnose.py --details --json > diagnosis.json
```

If openpilot is running in the usual tmux session, stop it with
`tmux kill-session -t comma` before scanning. The script leaves Panda in
no-output mode when it exits, including after errors or Ctrl-C. Restart
openpilot normally after the diagnostic session.

## Automatic prerequisite checks

Before sending diagnostic requests, the script checks Panda health/firmware
compatibility, ignition, and (for an internal comma Panda) the car harness.
Missing ignition, a missing harness, or an unreadable/incompatible Panda health
packet stops the scan with a setup error. Either the ignition line or CAN ignition
signal is sufficient. An external Panda without an ignition signal produces a
warning, since a direct OBD connection may not provide that signal.

Reported Panda fault flags produce a warning, not a scan-wide block. They are
telemetry rather than a diagnostic-permission check: for example, `faults=8`
records a CAN2 interrupt-rate fault that remains latched after the rate recovers.
The raw `faults` and `fault_status` remain in the report. A warning does not prove
the fault is historical or harmless; an underlying problem may still prevent
communication. Panda's own transmit restrictions remain enforced, and actual
connection failures are handled by the checks below.

For the bus-1 OBD route, a short connectivity check sends two standard read-only
OBD probes. Received traffic confirms CAN activity. No traffic together with
bus-off or new acknowledgement errors skips that route and points you to the
comma power OBD plug, its RJ45 cable to the harness box, ignition, and bitrate.
Other selected harness routes can still be scanned. A silent bus without new
errors remains unverified and is scanned with a warning.

**Panda cannot directly identify a connected comma power adapter.** Its power
reading and harness detection do not prove that comma power is installed. TX
counters and local echoes also do not prove another device acknowledged a frame.
The report therefore leaves `comma_power_present` null and reports measured OBD
connectivity separately. Equivalent diagnostic wiring can work without comma
power; some ECUs remain accessible through the camera/gateway harness alone.

## Targeting and routing

```sh
# Standard engine endpoint
python tools/scripts/car/diagnose.py --addr 0x7e0 --bus 1 --obd on

# Scan only the OBD port (avoids repeating discovery on harness routes)
python tools/scripts/car/diagnose.py --bus 1 --obd on

# Learn a target's reply address instead of supplying it
python tools/scripts/car/diagnose.py --addr 0x715 --bus 1 --obd on

# VW airbag: explicit physical request and response addresses
python tools/scripts/car/diagnose.py --addr 0x715 --rx-addr 0x77f --bus 1

# Subaddressed ECU, using the installed harness rather than the OBD port
python tools/scripts/car/diagnose.py --addr 0x750 --subaddress 0x0f --bus 0 --obd off
```

Default discovery scans buses 1, 0, and 2, trying both bus-1 OBD multiplexing
states. `--bus` is repeatable. When targeting, the default bus is 1; known opendbc
subaddress hints can add probes, but reply addresses are learned rather than
guessed. Supplying `--rx-addr` bypasses discovery for that explicit pair.
`--serial` selects one Panda when multiple are connected.

Discovery combines:

- Functional OBD PID 00 queries, retaining **all** 11-bit and 29-bit responders.
- Sequential UDS tester-present probes across `0x600`–`0x7ff` (excluding the
  functional address `0x7df`) and normal-fixed 29-bit `0x18daXXf1` addresses
  (excluding tester node `0xf1`), plus Panda's permitted `0x24b` exception. These
  stay within Panda's diagnostic safety restrictions; normal addressing needs no
  manufacturer module list.
- Actual reply addresses collected on the selected bus, followed by an independent
  read request to corroborate each request/reply pair. Identification DID `0xf197`
  is tried first, with a DTC-read fallback if needed. A valid negative response
  also confirms an endpoint; it does not mean that fault retrieval is supported.
  Functional OBD responders use the standardized emissions address mapping and
  a physical OBD follow-up.
- Optional opendbc address/subaddress hints, especially for subaddressed ECUs and
  addresses outside the generic ranges. Explicit `--subaddress` is also supported.

The scanner does **not** assume the general UDS reply address is request + `0x08`
or request + `0x6a`. It rejects queued/other-bus/echo traffic and corroborates
candidate pairs with a different read service. Delayed replies that cannot be
corroborated remain `unconfirmed`. Within a route, multiple reply addresses for
one request, or one reply address shared by multiple requests, remain `ambiguous`
and are not counted as separate identified ECUs. Their addresses and raw evidence
are in JSON. Results across different routes are still retained separately.
Avoid running any other diagnostic tester simultaneously: UDS replies lack a
transaction identifier, so this is bounded corroboration, not proof against
arbitrarily delayed or concurrent traffic.

Manufacturer metadata supplies optional hints and labels, not proof of ECU
identity or a required module inventory. The script does not run the firmware-query
sequences in those profiles. It does not enumerate every possible subaddress or
read a manufacturer's gateway installation list. Sleeping, inaccessible, or
nonresponding ECUs can still be missed; absence of a reply is not absence of a
module. See [the discovery research](diagnose-discovery-notes.md).

## Information returned

- OBD modes 03, 07, and 0A: stored, pending, and permanent codes, plus mode-01
  check-engine-light/monitor status on emissions endpoints.
- UDS service 19: DTCs and supported status bits. The DTC format is requested
  before rendering a P/C/B/U code; otherwise the three-byte identifier stays hex.
- `--details`: ECU identification, raw UDS snapshots and extended records, and
  supported OBD freeze-frame-zero values (load, coolant/intake temperature, RPM,
  speed, and the associated DTC).
- Optional descriptions from the bundled OBDex lookup table.

An OBD category such as `stored` is not a synthesized UDS status byte. Duplicate
codes from different protocols or routes retain their separate origins. UDS
failure-type bytes are retained. Unknown manufacturer codes remain usable even
without a description. ECU-specific snapshot/extended-record bodies remain raw;
interpreting them requires the correct manufacturer definitions. Snapshot values
are historical, not current sensor readings.

Only fixed read requests, tester-present, and default/extended diagnostic session
selection are allowed. Extended sessions are entered only after a session-related
rejection and a return to the default session is attempted afterward. There is no
code clearing, security unlocking, ECU reset, coding, or arbitrary-command option.

## JSON and incomplete scans

`--json` emits one report on stdout, with progress/debug output on stderr. Version 2
contains:

- `ecus`: physical addresses, route, identity candidates, decoded codes, and every
  attempted request with raw responses and its outcome.
- `discovery`: observed replies and confirmation queries, `unanswered` request
  addresses/subaddresses (no invented reply address), `unconfirmed` candidate
  pairs, `ambiguous` corroborated pairs, and the count `not_probed` before the
  deadline. An unanswered candidate is not proof that an ECU exists or is absent.
- `routes`: finished, incomplete, or unscanned routes.
- `preflight`: ignition/harness/health checks and measured evidence. An OBD
  route also has its own connectivity `preflight`; an unavailable route is
  marked `skipped`.
- `errors`, `warnings`, elapsed time, and whether the deadline was reached.
- `description_database`: source revision/license; per-code `lookup` fields are
  third-party descriptions, separate from vehicle-reported data.

`status: partial` means some DTC data was read. `status: failed` means none was
read, or setup failed. `vehicle_coverage_complete` is always false: this scanner
cannot establish a complete vehicle module inventory. An empty code list is only
meaningful for a successful query; a timeout/rejection is never reported as a clean
module or vehicle.

Exit codes: **0** = some DTC data read, **1** = no DTC data, **2** = setup/argument
error. Exit 0 does not mean the vehicle is fault-free or fully scanned. Scripts and
agents must inspect the report. Completed ECU results survive a later scan error
or interruption.

`--timeout` is an absolute per-request limit, including response-pending and
multi-frame reception (default 1 second). `--probe-timeout` controls the listen
window per discovery probe (default 0.1 seconds). Increase it for slow replies;
shorter windows trade coverage for speed. `--scan-timeout` bounds discovery/query
work (default 600 seconds, increased from 120 for sequential discovery). Expect
roughly 77 seconds of probe windows per full route, plus confirmations and fault
reads; scanning all four routes can take several minutes. Progress is printed
every 128 probes. Use `--bus 1 --obd on` for an OBD-port-only scan.
`--max-details` limits detailed retrieval to 16 UDS codes per ECU by default;
omitted details are counted. Increase these limits explicitly for slow ECUs or
large reports.

## Coverage

This is a cross-brand, best-effort **classic CAN OBD-II and UDS** reader. It does
not implement K-line, J1850, DoIP, CAN-FD diagnostics, or proprietary legacy fault
services. Wiring, gateway restrictions, ECU wake/session requirements, and CAN
bitrate can prevent access even on an openpilot-supported car. The installed
camera harness alone does not guarantee access to the airbag or engine ECU.
The normal Panda connection uses 500 kbit/s CAN. This implementation selects one
Panda; it does not automatically span multiple pandas.

Protocol behavior is tested with a simulated Panda and the real opendbc ISO-TP
implementation. Actual vehicle coverage still requires parked-car validation.

## Offline descriptions

`data/obdex.json` contains 9,533 English code titles from
[OBDex](https://github.com/foerbsnavi/OBDex), pinned to commit
`bc58b0eb7273226a1aabae98e956b70b8362bda1`. Upstream data is CC0-1.0; see
`data/LICENSE-OBDEX`. The file records SHA-256 hashes of its upstream inputs.

These are third-party labels, not verified diagnoses or repair recommendations.
Manufacturer-specific definitions are not covered. Causes, likelihoods, and repair
estimates are deliberately not included in the scanner's lookup table. Missing
or unreadable descriptions do not prevent fault retrieval.

To regenerate the table, use `python tools/scripts/car/data/update_obdex.py` in an
environment that already has PyYAML. This maintenance step downloads only the
pinned upstream data; the scanner itself makes no network requests and needs no
new runtime packages beyond the existing openpilot environment.

Run the hardware-independent tests from the checkout with its normal Python
environment:

```sh
python -m unittest tools.scripts.car.tests.test_diagnose
```
