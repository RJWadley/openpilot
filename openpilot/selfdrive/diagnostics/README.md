# Agent vehicle diagnostics

`diagnosticd` is an `always_run` Python process managed by openpilot. It serves MCP
at **`http://127.0.0.1:8766/mcp`**, onroad and offroad, but never scans at startup.
No model runtime or new Python dependency is required.

## Tools

- `scan_vehicle(target?, broad=false, fast=false, details=false)` returns the
  readable report and `scan_id`. An agent request is the approval; there is no
  on-device confirmation tap. Target is a physical CAN address such as `0x715`.
- `get_scan_evidence(scan_id="latest", ecu?)` reads the saved readable report and
  raw evidence, without touching the car. ECU filtering is recommended for small
  model context windows. The filter retains every route matching that address.

OBD-only discovery is the default. `broad` adds harness routes. The existing
vehicle-verified cache behavior is unchanged. Fault/history interpretation, full
OBDex entries, searchable unknown codes, and incomplete coverage are preserved.
There are no code-clearing, ECU-coding, security-unlock, shell, or arbitrary-CAN
tools. Session-control requests used by the reader remain narrowly allowlisted.

Both the MCP tools and `tools/scripts/car/diagnose.py` call the same scanner.
CLI defaults to coordinated access; `--direct` explicitly selects the older
standalone Panda workflow and still refuses while pandad is running.

## Safe scan lifecycle

1. The car must be recognized and initialized, ignition on, disengaged, and
   stationary in Park (or Neutral with parking brake). Missing or stale state is
   a refusal, not an assumption of zero speed. Currently one Panda is supported.
   Passive/dashcam or mock-car operation without normal safety initialization is
   not supported by this first coordinated implementation.
2. `pandad` announces preparation. `selfdrived` adds an actual no-entry/immediate-
   disable event and a visible diagnostic alert. `card` pauses normal control TX.
   Both must acknowledge the current session and route before ELM327 is selected.
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

MCP scan responses use POST SSE with keepalives/progress. A dropped HTTP connection
does not cancel a scan: retrieve `latest` evidence after it finishes. Explicit MCP
cancellation or session deletion asks the reader to stop and recover. A repeated
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
scons openpilot/selfdrive/pandad/tests/test_diagnostic_session
openpilot/selfdrive/pandad/tests/test_diagnostic_session
```

Tests include real HTTP/SSE, real Cereal/msgq, the existing ISO-TP engine, and
simulated ECUs. Native session-policy tests cover gating/recovery. Those are not
live vehicle, actual onroad-process restart, or ChatGPT-through-tunnel proof.
First deployment requires a supervised parked-car check of entry refusal,
engagement lockout, scan results, client termination, and successful restoration.
