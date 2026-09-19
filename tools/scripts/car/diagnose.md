# Diagnostic reader

Run on an installed comma with the vehicle parked, ignition on, and openpilot
stopped. This script takes direct ownership of the Panda. It refuses to run while
`pandad` is running and does not stop or restart openpilot for you.

From the openpilot checkout on the comma:

```sh
python tools/scripts/car/diagnose.py
python tools/scripts/car/diagnose.py --fast
python tools/scripts/car/diagnose.py --broad
python tools/scripts/car/diagnose.py --broad --fast
python tools/scripts/car/diagnose.py --json > diagnosis.json
# Keep raw protocol evidence separately; choose a new filename each run.
python tools/scripts/car/diagnose.py --details --evidence evidence.json --json > diagnosis.json
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
The raw `faults` and `fault_status` remain in the technical evidence. A warning does not prove
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

Default discovery scans **only bus 1 with OBD multiplexing on**. `--broad` also
scans bus 1 with multiplexing off and harness buses 0 and 2. Harness access is
opt-in; a disconnected OBD route does not automatically enable it.

`--bus` is repeatable and overrides the selected buses, including with `--broad`.
`--obd` overrides bus-1 routing: `on` selects OBD, `off` selects the harness, and
`auto` tries both. Targeting keeps these same route defaults. Known opendbc
subaddress hints can add probes, but reply addresses are learned rather than
guessed. Supplying `--rx-addr` bypasses discovery and the module cache for that explicit pair.
`--serial` selects one Panda when multiple are connected.

## Fast scans and the module cache

A completed, untargeted discovery saves confirmed module addresses, routes,
emissions capability, and discovery timestamps. It does **not** save fault codes.
`--fast` reuses that inventory and reads fresh faults from every cached target
on the selected routes. `--fast --broad` includes cached harness routes too;
`--fast` alone stays OBD-only even if the cache came from a broad scan.

Before reusing an inventory, the scanner reads the VIN from the car again using
UDS DID `0xF190` or OBD mode `09`, PID `02`, and compares its hash with the cached
vehicle identity. It does not trust openpilot's saved `CarVin` parameter or the
Panda serial as vehicle identification. Only the VIN hash is saved, not the raw VIN.
If VIN verification is unavailable, the vehicle changed, the cache is missing or
invalid, or a selected route was not cached, that route uses normal discovery.
A first run with `--fast` therefore still performs discovery.

The latest vehicle's cache lives at `/data/diagnostics/modules.json` on a comma,
or `~/.cache/openpilot/diagnostics/modules.json` on a PC. It survives reboots and
is atomically replaced. Interrupted, deadline-limited, targeted, or ambiguous
discovery does not replace that route's cached inventory. A failed cache read
or write is reported without discarding diagnostic results. A new vehicle's
inventory replaces the previous vehicle's; narrower scans of the same vehicle
preserve previously cached routes outside the selected scope.

Cached modules that no longer respond are reported as unavailable, not healthy.
New or previously missed modules will not appear on reused routes: run without
`--fast` to refresh discovery. Even a completed discovery is only best-effort,
not proof of a complete vehicle inventory. JSON reports include `cache` metadata
and each route's `source` (`discovery`, `cache`, or `explicit`); cached routes
also include their original `cached_at` timestamp.

## Discovery

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
and are not counted as separate identified ECUs. Their counts remain in the diagnosis
report; their addresses and raw replies are saved with `--evidence FILE`.
Results across different routes are still retained separately.
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
  before rendering a P/C/B/U code; otherwise the three-byte identifier stays raw.
- Records without supported failure, pending, confirmed, failure-history, or
  warning-indicator flags are omitted from `codes`. In particular, test-not-completed
  flags alone are not faults. `ignored_non_fault_records` counts omitted UDS records;
  the count and original response bytes remain in the technical evidence.
- Every code keeps its original `status` flags and adds a deterministic, plain-English
  `status_summary`. A stored or confirmed code is not proof of a problem happening
  now. A failed latest test is not a continuous live measurement. We do not invent
  an `active` classification from history flags or treat an absent flag as proof
  of health. UDS status semantics follow the
  [AUTOSAR Diagnostic Event Manager definitions](https://www.autosar.org/fileadmin/standards/R23-11/CP/AUTOSAR_CP_SWS_DiagnosticEventManager.pdf).
- ECU identification is requested for modules returning faults, even without
  `--details`. Supported OBD freeze-frame-zero values (load, coolant/intake
  temperature, RPM, speed) are also read automatically when OBD codes are returned.
  The short, decoded `freeze_frame` is attached only to the matching OBD code
  from that same ECU, with units and `historical: true`. It is not attached to an
  unknown-format UDS code just because both codes might describe the same problem.
  Missing, unsuccessful, unassociated, and undecoded readings are not included;
  missing data is not a zero value or proof of a healthy system.
- `--details` additionally reads raw UDS snapshots/extended records and identifies
  modules without faults. Save these technical details with `--evidence FILE`;
  they are not injected into the model-facing report. The detail limit applies
  after filtering non-faults.
- Full available OBDex entries under `lookup.entry`, with the source and English
  title also available under `lookup.source` and `lookup.title`. Every upstream field,
  translation, nested value, and reference is retained. The text report prints the
  complete matched entry too. Missing upstream fields are not invented.

An OBD category such as `stored` is not a synthesized UDS status byte. Duplicate
codes from different protocols or routes retain their separate origins. UDS
failure-type bytes are retained. Unknown manufacturer codes remain usable even
without a description. ECU-specific snapshot/extended-record bodies remain raw;
interpreting them requires the correct manufacturer definitions. Snapshot values
are historical, not current sensor readings.

For faults without OBDex entries, `display_code` and `search` provide copyable search
forms. Known SAE codes retain their P/B/C/U notation and failure type. Unknown-format
UDS codes include the complete raw integer in decimal and hexadecimal; no inferred
SAE code or manufacturer mapping is used. For example, raw `0x901614` is also
`9442836`, a form found in [VCDS scan logs](https://forums.ross-tech.com/index.php?threads/21439/).
Faults trigger read-only ECU identification queries even without `--details`.
Successful identifiers are exposed under `identity`; `search.query` combines the
code with the ECU-reported component and part number when available, e.g.
`9442836 AirbagVW20 5Q0959655J`. These searches are suggestions, not verified meanings.
The canonical `code` remains in the diagnosis report; `raw_dtc`, status bytes,
availability masks, and reported `format` remain in the technical evidence.

Only fixed read requests, tester-present, and default/extended diagnostic session
selection are allowed. Extended sessions are entered only after a session-related
rejection and a return to the default session is attempted afterward. There is no
code clearing, security unlocking, ECU reset, coding, or arbitrary-command option.

## JSON and incomplete scans

Text and JSON use the same diagnosis view. `--json` emits one report on stdout,
with progress/debug output on stderr. Schema version **4**, `report_kind: diagnosis`,
contains:

- `ecus`: physical addresses, route, ECU-reported identity, filtered fault/history
  codes with plain-English status and full OBDex entries, and associated decoded
  historical context. Address-based identity guesses are excluded.
- `read_results`: per-ECU outcomes of fault-code requests, including timeouts and
  unsupported services. Optional identification/format/snapshot request failures
  are left in technical evidence. Reading one service successfully does not hide
  failed attempts to read another.
- `emissions_status`: per-ECU check-engine-light request and stored-code count,
  only when the module answered successfully. `source: ecu` and `timing: at_scan`
  distinguish this from OBDex reference flags and historical snapshots.
- `summary`: listed module endpoints, endpoints with/without DTC data, and
  fault/history record count. Neither endpoints across routes nor codes across
  protocols are assumed to be unique physical modules or distinct problems.
- `discovery`: per-route counts of probes, unanswered requests, unconfirmed and
  ambiguous reply pairs, and unprobed candidates. An unanswered candidate is not
  proof that an ECU exists or is absent.
- `routes`: finished, incomplete, or unscanned routes.
- `preflight`: ignition/harness/health check outcomes and messages. An OBD
  route also has its own connectivity `preflight`; an unavailable route is
  marked `skipped`.
- `errors`, `warnings`, elapsed time, and whether the deadline was reached.
- `description_database`: source revision/license; per-code `lookup.entry` fields
  are complete third-party reference entries, separate from vehicle-reported data.

Compared with version 3, raw requests/responses, decoded query duplication,
discovery address lists, hardware telemetry, and address-based identity guesses
are no longer included in the main report. Full OBDex entries are still retained.

`--evidence FILE` writes a separate JSON report with `report_kind: technical_evidence`
and the same schema version. It preserves the detailed scan structure from
version 3: `ecus[].queries`, original response bytes, raw UDS records (if requested),
identity candidates, omitted-detail/non-fault counts, discovery addresses/replies,
and preflight telemetry. This file is written after Panda cleanup and is **not**
automatically included in model input. It refuses to overwrite an existing file;
an evidence-write failure warns without discarding the diagnosis. On success,
the main report includes `evidence_file` with its path. Without this option, raw
evidence is not persisted. Cache behavior is unchanged.

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
`--max-details` limits detailed retrieval to 16 UDS fault/history records per ECU by default;
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

`data/obdex.json.gz` contains 9,533 complete code entries from
[OBDex](https://github.com/foerbsnavi/OBDex), pinned to commit
`bc58b0eb7273226a1aabae98e956b70b8362bda1`. Upstream data is CC0-1.0; see
`data/LICENSE-OBDEX`. The file records SHA-256 hashes of its upstream inputs.

The entries include available explanations, components, possible causes,
likelihoods, symptoms, repair estimates, flags, and references. These are third-party
reference data, not confirmed diagnoses or repair recommendations. For example,
OBDex's `flags.mil` is not the live vehicle's warning-light state, and its repair
estimate is not a quote for this car. Manufacturer-specific definitions are not
covered, and not every entry supplies every optional field. Missing or unreadable
reference data does not prevent fault retrieval.

The former title-only `obdex.json` is superseded by the compressed complete dataset.
Compression is deterministic, and the updater verifies every pinned source hash
and the full JSON round trip. Runtime decompression uses Python's standard library.

To regenerate the table, use `python tools/scripts/car/data/update_obdex.py` in an
environment that already has PyYAML. This maintenance step downloads only the
pinned upstream data; the scanner itself makes no network requests and needs no
new runtime packages beyond the existing openpilot environment.

Run the hardware-independent tests from the checkout with its normal Python
environment:

```sh
python -m unittest tools.scripts.car.tests.test_diagnose tools.scripts.car.tests.test_update_obdex
```
