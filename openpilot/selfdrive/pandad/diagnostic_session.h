#pragma once

#include <cstdint>
#include <string>

// Pure session policy. Hardware configuration and message freshness are supplied
// by pandad; no model or HTTP client gets to override these gates.
class DiagnosticSession {
public:
  enum class Phase { idle, preparing, scanning, restoring };
  struct Input {
    uint64_t now = 0;
    uint64_t id = 0;
    uint32_t route = 0;
    bool active = false;
    bool obd = true;
    bool fresh_request = false;
    bool safe = false;
    bool acknowledged = false;
    bool elm_ready = false;
    bool normal_ready = false;
    bool onroad = false;
    bool observed_offroad = false;
  };

  Phase phase = Phase::idle;
  uint64_t id = 0, changed_at = 0, started_at = 0, last_id = 0;
  uint32_t route = 0;
  bool obd = true, saw_offroad = false;
  std::string error;

  void recover(uint64_t now, const std::string &reason = "") {
    phase = Phase::restoring;
    changed_at = now;
    saw_offroad = false;
    error = reason;
  }

  void update(const Input &in) {
    if (phase == Phase::restoring) {
      saw_offroad |= in.observed_offroad;
      if (saw_offroad && in.onroad && in.safe && in.normal_ready && in.acknowledged) {
        phase = Phase::idle;
        changed_at = in.now;
      }
      return;
    }
    if (phase == Phase::idle) {
      if (in.fresh_request && in.active && in.id && in.id != last_id) {
        last_id = id = in.id;
        error.clear();
        if (!in.safe || !in.normal_ready) {
          error = "Vehicle must be parked, disengaged, ignition on, and openpilot initialized with fresh vehicle state";
          return;
        }
        route = in.route;
        obd = in.obd;
        phase = Phase::preparing;
        changed_at = started_at = in.now;
      }
      return;
    }
    if (in.now - started_at > 660000000000ULL) {
      recover(in.now, "Maximum diagnostic session duration exceeded");
      return;
    }
    if (!in.safe || !in.fresh_request || in.id != id || !in.active) {
      recover(in.now, !in.safe ? "Vehicle state became unsafe or unavailable" :
              (!in.fresh_request ? "Diagnostic client heartbeat expired" : ""));
      return;
    }
    if (route != in.route || obd != in.obd) {
      route = in.route;
      obd = in.obd;
      phase = Phase::preparing;
      changed_at = in.now;
      return;  // The old route's acknowledgements cannot authorize the new route.
    }
    if (phase == Phase::preparing) {
      if (in.now - changed_at > 5000000000ULL) {
        recover(in.now, "Timed out acquiring diagnostic interlocks or Panda routing");
      } else if (in.acknowledged && in.elm_ready) {
        phase = Phase::scanning;
      }
    } else if (!in.acknowledged || !in.elm_ready) {
      recover(in.now, "Diagnostic interlock or Panda configuration lost");
    }
  }

  bool normal_tx() const { return phase == Phase::idle || (phase == Phase::restoring && saw_offroad); }
  bool diagnostic_tx() const { return phase == Phase::scanning; }
};
