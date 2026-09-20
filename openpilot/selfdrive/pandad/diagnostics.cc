#include "selfdrive/pandad/diagnostics.h"

#include <cmath>
#include <cstring>

#include "selfdrive/pandad/diagnostic_requests.h"

PandaDiagnostics::PandaDiagnostics(Panda *panda) : panda_(panda) {
  if (params_.getBool("DiagnosticRecoveryRequired")) session_.recover(nanos_since_boot(), "Recovering interrupted diagnostics");
}

bool PandaDiagnostics::fresh(SubMaster &sm, const char *name, uint64_t now, uint64_t age) {
  const auto t = sm[name].getLogMonoTime();
  return sm.valid(name) && t && t <= now && now - t < age;
}

bool PandaDiagnostics::acknowledged(SubMaster &sm, uint64_t now) {
  auto card = sm["diagnosticCardAck"].getDiagnosticCardAck();
  auto controls = sm["diagnosticControlsAck"].getDiagnosticControlsAck();
  return fresh(sm, "diagnosticCardAck", now) && fresh(sm, "diagnosticControlsAck", now) &&
         sm["diagnosticCardAck"].getLogMonoTime() >= session_.changed_at &&
         sm["diagnosticControlsAck"].getLogMonoTime() >= session_.changed_at &&
         card.getSessionId() == session_.id && controls.getSessionId() == session_.id &&
         card.getRoute() == session_.route && controls.getRoute() == session_.route;
}

void PandaDiagnostics::update(SubMaster &sm, PandaSafety &safety, bool onroad) {
  std::lock_guard guard(lock_);
  uint64_t now = nanos_since_boot();
  auto health = panda_->get_state();
  auto cs = sm["carState"].getCarState();
  auto req = sm["diagnosticRequest"].getDiagnosticRequest();
  // Neutral with the parking brake supports manual cars without a Park gear.
  bool parked = cs.getGearShifter() == cereal::CarState::GearShifter::PARK ||
                (cs.getGearShifter() == cereal::CarState::GearShifter::NEUTRAL && cs.getParkingBrake());
  bool safe = fresh(sm, "carState", now, 250000000ULL) && cs.getCanValid() && cs.getStandstill() &&
              std::isfinite(cs.getVEgo()) && std::abs(cs.getVEgo()) < 0.1 && parked &&
              fresh(sm, "selfdriveState", now) && !sm["selfdriveState"].getSelfdriveState().getEnabled() &&
              fresh(sm, "deviceState", now, 2000000000ULL) && onroad && health && panda_->comms_healthy() &&
              (health->flags_pkt & (HEALTH_FLAG_IGNITION_LINE | HEALTH_FLAG_IGNITION_CAN)) &&
              health->car_harness_status_pkt != 0 && !(health->flags_pkt & HEALTH_FLAG_CONTROLS_ALLOWED);
  bool elm = health && elm_requested_at_ &&
             health->safety_mode_pkt == (uint16_t)cereal::CarParams::SafetyModel::ELM327 &&
             health->safety_param_pkt == (session_.obd ? 0 : 1);
  auto previous = session_.phase;
  auto previous_route = session_.route;
  session_.update({now, req.getSessionId(), req.getRoute(), req.getActive(), req.getObd(),
                   fresh(sm, "diagnosticRequest", now), safe, acknowledged(sm, now), elm,
                   health && safety.matches(*health) && (session_.phase != DiagnosticSession::Phase::restoring || cycle_requested_),
                   onroad, fresh(sm, "deviceState", now, 2000000000ULL) && !onroad});
  if (previous != session_.phase || previous_route != session_.route) {
    tx_after_ = now;
    if (session_.phase != DiagnosticSession::Phase::scanning) elm_requested_at_ = 0;
  }

  using Phase = DiagnosticSession::Phase;
  if (session_.phase == Phase::preparing && acknowledged(sm, now) && safe && !elm_requested_at_) {
    // Durable before changing hardware; a pandad/manager restart must recover too.
    if (params_.putBool("DiagnosticRecoveryRequired", true) != 0) {
      session_.recover(now, "Could not persist diagnostic recovery state");
    } else {
      panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, session_.obd ? 0U : 1U);
      elm_requested_at_ = now;
    }
  }
  if (session_.phase == Phase::restoring && !recovery_started_) {
    params_.putBool("DiagnosticRecoveryRequired", true);
    panda_->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
    recovery_started_ = true;
    cycle_requested_ = false;
    tx_after_ = now;
  }
  if (session_.phase == Phase::restoring && !cycle_requested_ && now - session_.changed_at >= 6000000000ULL) {
    // Give abandoned diagnostic sessions a quiet timeout before restarting
    // ordinary onroad initialization (never pandad or the MCP process).
    if (params_.putBool("OnroadCycleRequested", true) == 0) {
      cycle_requested_ = true;
      session_.saw_offroad = false;  // Require a fresh cycle after this request.
    } else {
      session_.error = "Could not request normal-operation recovery; engagement remains blocked";
    }
  }
  if (session_.phase == Phase::idle || (session_.phase == Phase::restoring && session_.saw_offroad)) {
    safety.configureSafetyMode(onroad);
  }
  if (previous == Phase::restoring && session_.phase == Phase::idle) {
    if (params_.putBool("DiagnosticRecoveryRequired", false) == 0) {
      recovery_started_ = false;
    } else {
      session_.phase = Phase::restoring;
      session_.error = "Could not clear recovery state; engagement remains blocked";
    }
  }
  tx_until_ = safe && acknowledged(sm, now) ? now + 200000000ULL : 0;
  publish();
}

void PandaDiagnostics::publish() {
  MessageBuilder msg;
  auto state = msg.initEvent().initDiagnosticState();
  state.setSessionId(session_.id);
  state.setRoute(session_.route);
  state.setObd(session_.obd);
  state.setPhase(static_cast<cereal::DiagnosticState::Phase>(session_.phase));
  state.setError(session_.error);
  pm_.send("diagnosticState", msg);
}

void PandaDiagnostics::send(cereal::Event::Reader event, bool diagnostic, bool fake_send) {
  std::lock_guard guard(lock_);
  uint64_t now = nanos_since_boot(), sent = event.getLogMonoTime();
  if (fake_send || sent > now || sent < tx_after_) return;
  if (!diagnostic) {
    const bool recovery_ready = session_.phase != DiagnosticSession::Phase::restoring || cycle_requested_;
    if (session_.normal_tx() && recovery_ready && now - sent < 1000000000ULL) panda_->can_send(event.getSendcan());
    return;
  }
  auto tx = event.getDiagnosticSendcan();
  if (!event.getValid() || !session_.diagnostic_tx() || now >= tx_until_ || now - sent >= 200000000ULL ||
      tx.getSessionId() != session_.id || tx.getRoute() != session_.route || tx.getFrames().size() > 32) return;
  for (auto frame : tx.getFrames()) {
    auto data = frame.getDat();
    if (!diagnostic::frame(frame.getAddress(), data.begin(), data.size(), frame.getSrc()) || (session_.obd && frame.getSrc() != 1)) return;
  }
  MessageBuilder msg;
  auto frames = msg.initEvent().initSendcan(tx.getFrames().size());
  size_t i = 0;
  for (auto frame : tx.getFrames()) {
    frames[i].setAddress(frame.getAddress());
    frames[i].setDat(frame.getDat());
    frames[i++].setSrc(frame.getSrc());
  }
  panda_->can_send(frames.asReader());
}
