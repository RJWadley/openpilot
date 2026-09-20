# Agent vehicle diagnostics

`diagnosticd` is an `always_run` Python process managed by openpilot. It serves MCP
at **`http://127.0.0.1:8766/mcp`**, onroad and offroad, but never scans at startup.
No model runtime or new Python dependency is required.

## Tools

- `scan_vehicle(target?, details=false, wait=true)` starts
  a fresh OBD-only scan and waits for completion, returning the first compact report
  page. `wait=false` instead returns its `scan_id` and lifecycle state immediately. An agent request
  is the approval; there is no on-device confirmation tap. Target is a physical CAN
  address such as `0x715`. A busy result includes `active_scan` and its ID; inspect
  that operation instead of retrying the scan.
- `get_scan_status(scan_id="active")` returns phase, progress, start/update/finish
  timestamps, execution state, coverage, `report_ready`, and restoration state.
  It never starts a scan or contacts the car. `active` selects the current/last
  operation; `latest` selects the latest compatible saved report's operation.
- `get_scan_report(scan_id="latest", ecu?, cursor?, limit=20)` reads compact
  findings: ECU identities, exact codes, status explanations, OBDex titles, counts,
  decoded historical context and warnings. No raw replies or discovery-address lists.
- `get_scan_evidence(scan_id="latest", ecu?, raw=false, cursor?, limit=20)` uses
  the same compact default. Set `raw=true` for raw replies, discovery evidence and
  **full, unchanged OBDex entries**. This does not run additional vehicle queries;
  collecting optional snapshot/extended records requires `details=true` when scanning.
  Leave `details=false` for routine scans; these extra queries add time.

Reading a known scan's report before publication returns its lifecycle state,
`report_ready=false` and `next_tool=get_scan_status`, not a filesystem error.
Unknown/pruned IDs produce a readable error. Neither case starts another scan.

Both report tools are bounded to **8,000 serialized JSON bytes per page** and
1–50 records (`limit`). MCP's text and structured result representations duplicate
that page on the wire. Follow `next_cursor` until null; the first page may not
contain all faults. ECU filters apply to both compact and raw views, retaining all
routes matching the transmit address. Summary counts always describe the entire
scan. Cursors pin the original scan, even if another report becomes `latest`;
keep the same ECU filter and raw setting. A pruned report's cursor returns an error.

An oversized record becomes `json_fragment` items: concatenate `text` in `offset`
order until `final=true`, then parse the JSON record. `total_chars` and `record_index`
identify the fragment sequence. Raw records include a `path` within the filtered
evidence document and its full `value`. No raw record is silently truncated.

`latest` means latest **compatible saved report**, not necessarily the running scan. Responses
include `report_age_seconds` (since `started_at`), `active_scan_id` and `is_active_scan`.
Unavailable lifecycle evidence is reported as restoration `unknown`.

Every new report bundle, its readable report, and its raw evidence carry
`server_version`, from the same `SERVER_VERSION` constant in `version.py` used by
MCP initialization. Reads require an exact match. `latest` skips reports with a
missing/different version; explicit IDs and pagination cursors return a clear
incompatibility error. If none match, the tools explain that a new scan needs an
explicit request; they never scan automatically. Compatibility checks do not
delete, rewrite, or migrate old files; ordinary storage retention still applies.
`schema_version` describes the diagnostic data format and remains separate from
the producer version and negotiated MCP protocol version. Bump `SERVER_VERSION`
for server releases to invalidate reports from previous implementations.

MCP always performs fresh OBD-only discovery. `broad` and `fast` are no longer MCP
arguments; stale calls containing them are rejected before any vehicle access.
The CLI retains `--broad` and `--fast` for advanced use. Fault/history interpretation,
full OBDex entries, searchable unknown codes, and coverage limits are preserved.
There are no code-clearing, ECU-coding, security-unlock, shell, or arbitrary-CAN
tools. Session-control requests used by the reader remain narrowly allowlisted.

Both the MCP tools and `tools/scripts/car/diagnose.py` call the same scanner.
CLI defaults to coordinated access; `--direct` explicitly selects the older
standalone Panda workflow and still refuses while pandad is running.

Tool descriptions instruct agents to show the exact DTC and ECU identity, use the
supplied search query for missing descriptions when web search is available, cite
trustworthy matches and state uncertainty. Code formatting is independent of
description lookup: preserve raw DTCs and only use standardized display formatting
when the encoding is established. Stored/confirmed is not proof of a current fault;
freeze frames are historical, references are not diagnoses, and missing data is
not evidence of health.

## Safe scan lifecycle

1. The car must be recognized and initialized, ignition on, disengaged, and
   stationary in Park (or Neutral with parking brake). Missing or stale state is
   a refusal, not an assumption of zero speed. Currently one Panda is supported.
   Passive/dashcam or mock-car operation without normal safety initialization is
   not supported by this first coordinated implementation.
2. `pandad` announces preparation. `selfdrived` adds an actual no-entry/immediate-
   disable event and a visible diagnostic alert. `card` pauses normal control TX.
   Both must acknowledge the current session and route before ELM327 is selected.
   Tell users the comma screen shows diagnostic/engagement-block status, not detailed
   per-ECU progress. The banner outranks routine permanent LKAS/cruise fault banners
   but retains those faults in its text. Higher-priority safety alerts still win;
   no safety events are suppressed. Preparation/restoration have distinct labels.
3. `pandad` remains the sole hardware reader/configuration owner. The scanner
   subscribes to copies of `can` and publishes a separate, session-tagged
   diagnostic TX stream. It never clears Panda's shared receive queue. Native TX
   gates reject old sessions, old routes, stale frames, and non-allowlisted requests.
4. A route change requires another acknowledgement. Motion, invalid state,
   ignition loss, missing acknowledgements, unexpected safety configuration,
   client heartbeat loss (500 ms), or the 11-minute native hard limit terminates
   diagnostic TX. The normal scan budget is 10 minutes.
5. Finish/cancel/failure enters recovery. A persistent latch survives process and
   manager restarts. Panda is put into no-output, followed by a six-second quiet
   interval and the existing offroad/onroad cycle to rerun normal car initialization.
   This briefly restarts the onroad processes/UI flow, **not pandad or MCP**.
   Engagement stays blocked until a fresh offroad transition, normal safety
   configuration, valid parked state, and consumer acknowledgements are observed.

Keep the car parked throughout. The client waits up to 60 seconds for recovery;
if it cannot verify completion, it returns `recovery_required` and the device
remains locked out. Do not manually remove `DiagnosticRecoveryRequired` to bypass
an unresolved recovery. Check connections and restart openpilot/the device.
An idle MCP frontend failure alone is not a driving-process fault.

Execution and coverage are separate. A completed collection is `status=complete`
when discovered modules returned DTC data and no concrete collection gaps remain;
its MCP `coverage=best_effort` and `vehicle_coverage_complete=false` still make no
claim about undiscovered modules. `partial` is reserved for actual gaps: unread
modules, unfinished routes/discovery, ambiguous or unconfirmed reply pairs,
deadline expiry or collection errors. Unsupported optional queries alone do not
make an otherwise successful scan partial. Compact pages carry `coverage_gaps`
counts and summary warning/error counts; warning/error records precede findings.
Agents should lead with completion, read-module counts and faults, mention concrete
gaps, and avoid repeating a generic partial-results disclaimer.

Coverage can also be `unknown` before collection or `unavailable` when no DTC data
was obtained. `collection_finished_at` does **not** mean restoration
has completed. During cleanup, phase is `restoring`, execution remains `running`,
and restoration state is `in_progress`. A fresh native coordinator idle response
allows `restoration.state=verified` (normal openpilot operation restored).
`not_needed` means this scan never requested diagnostic mode; it is not a general
openpilot health check. A timeout/error yields `unverified`. A stopped worker with
no terminal evidence yields `interrupted` and restoration `unknown`, never an
assumed successful recovery. The status is a recorded observation, not a live
vehicle-safety guarantee.

These checks do not establish universal ECU coverage or prove that a car is safe
to drive. Wiring and gateway/session restrictions still apply. ECU diagnostic
session state cannot be universally verified; the quiet interval and normal
initialization are recovery precautions, not a guarantee about every ECU.

## Storage and concurrency

The latest 20 report/evidence bundles survive restarts, subject to a 100 MiB total
cap. On comma they live in `/data/diagnostics/reports`; on a PC the location is
`~/.cache/openpilot/diagnostics/reports`. Oldest owned reports are pruned first.
Bundles are atomically published with exclusive names and private file permissions.
One cross-process lock excludes simultaneous CLI/MCP scans.

Lifecycle records also survive restarts and are bounded to the latest 20 operations.
Live progress is persisted at phase transitions and at most once a second within
a phase; the server keeps the current in-memory state. `seconds_since_update`
exposes the age of the last update, not an estimated percentage or time remaining.

Default scan calls use `wait=true`: POST SSE keeps the
request open and returns the first compact report page at completion. If the caller
supplies `_meta.progressToken`, native `notifications/progress` carry increasing
sequence values and real phase messages: discovery addresses checked, ECUs being
read, current ECU address/reported identity, collection complete and restoration.
Counts are per route and phase, not a vehicle-wide completeness percentage.
Notifications stop at the final response. Without a token, SSE uses keepalives.
An immediate-return request cannot continue emitting native progress after its
response; poll `get_scan_status` instead. Experimental MCP Tasks are not required
or advertised. ChatGPT's display/model exposure of notifications is not verified.

A dropped HTTP connection does not cancel a scan: inspect its ID afterward, or
use `get_scan_status("active")` if the client has not received the ID yet. Never
retry `scan_vehicle` merely to check progress. Clients with short tool-call timeouts
can explicitly choose `wait=false` and poll status instead of holding a connection.
Explicit MCP cancellation applies only while the `wait=true` request is pending;
session deletion or server shutdown also asks owned scans to stop and restore.
A late cancellation for an already-returned async request is ignored. A repeated
request ID in the same session reuses its job while retained (latest 20 completed
jobs); it is not a durable global idempotency key.

## Build and local use

Rebuild before restarting openpilot: this change adds Cereal schemas, native
`pandad` code, and a Params key. On the comma use the checkout's normal `scons`
build, then restart openpilot while parked. Do not hot-swap only the Python files.

For local MCP development without manager:

```sh
python -m openpilot.selfdrive.diagnostics.mcp
```

The endpoint implements MCP Streamable HTTP versions `2025-03-26`, `2025-06-18`,
and `2025-11-25`, negotiated at initialization. It exposes only the tools
capability. Preserve the returned `MCP-Session-Id`; send
`notifications/initialized` before tool calls. GET streams/resumability are not
offered (GET returns 405). Version negotiation does not claim implementation of
newer revisions. See the [MCP transport specification](https://modelcontextprotocol.io/specification/2025-11-25/basic/transports).

## Tunnel and access control: separate setup

Loopback binding is mandatory. Nothing here creates a tunnel or exposes the
vehicle publicly. A tunnel URL is **not account authentication**. Configure and
verify an authenticated HTTPS proxy/client connection before exposing this to
ChatGPT or any remote model. There is no per-user pairing/OAuth flow in this demo.
Do not assume a browser's Cloudflare login cookie authenticates an MCP client.

Environment settings, inherited from openpilot manager:

| Setting | Default | Meaning |
| --- | --- | --- |
| `DIAGNOSTIC_MCP_PORT` | `8766` | Loopback listening port |
| `DIAGNOSTIC_MCP_HOSTS` | empty | Additional allowed Host names, comma-separated; no scheme/port |
| `DIAGNOSTIC_MCP_ORIGINS` | empty | Exact allowed Origin values; present but unlisted Origins are rejected |
| `DIAGNOSTIC_MCP_TOKEN` | unset | Optional single-user bearer token for clients/proxies that can supply one |

The token is not an OAuth/pairing implementation and is not a promise of ChatGPT
authentication compatibility. Keep credentials out of source and logs. Headers,
request bodies, and reports are not emitted by the HTTP access logger. Saved
evidence can contain vehicle identifiers; protect the device and tunnel.

## Verification

```sh
python tools/test_runner.py -j1 tools/scripts/car/tests/test_diagnose.py tools/scripts/car/tests/test_update_obdex.py openpilot/selfdrive/diagnostics/tests
python tools/test_runner.py -j1 openpilot/selfdrive/selfdrived/tests/test_diagnostic_alerts.py
scons openpilot/selfdrive/pandad/tests/test_diagnostic_session
openpilot/selfdrive/pandad/tests/test_diagnostic_session
```

Tests include real HTTP/SSE, real Cereal/msgq, the existing ISO-TP engine, and
simulated ECUs, compact/raw paging and reconstruction, ECU filters, busy/cancel
states, worker termination, persistence, and restoration/storage failures.
Native session-policy tests cover gating/recovery. Those are not
live vehicle, actual onroad-process restart, or ChatGPT-through-tunnel proof.
First deployment requires a supervised parked-car check of entry refusal,
engagement lockout, scan results, client termination, and successful restoration.
