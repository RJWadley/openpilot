#pragma once

#include <mutex>

#include "selfdrive/pandad/diagnostic_session.h"
#include "selfdrive/pandad/pandad.h"
#include "openpilot/cereal/messaging/messaging.h"

class PandaDiagnostics {
public:
  explicit PandaDiagnostics(Panda *panda);
  void update(SubMaster &sm, PandaSafety &safety, bool onroad);
  void send(cereal::Event::Reader event, bool diagnostic, bool fake_send);

private:
  bool fresh(SubMaster &sm, const char *name, uint64_t now, uint64_t age = 500000000ULL);
  bool acknowledged(SubMaster &sm, uint64_t now);
  void publish();

  Panda *panda_;
  Params params_;
  PubMaster pm_{ {"diagnosticState"} };
  DiagnosticSession session_;
  std::mutex lock_;
  uint64_t tx_after_ = 0, tx_until_ = 0, elm_requested_at_ = 0;
  bool recovery_started_ = false, cycle_requested_ = false;
};
