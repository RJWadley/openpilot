#include <iostream>

#include "common/tests/native_test.h"
#include "selfdrive/pandad/diagnostic_session.h"
#include "selfdrive/pandad/diagnostic_requests.h"

using Session = DiagnosticSession;
using Phase = Session::Phase;

Session::Input ready(uint64_t now = 1000000000ULL) {
  return {now, 123, 1, true, true, true, true, false, false, true, true};
}

int main() {
  // No unsafe, stale, uninitialized, or zero-ID client can acquire a session.
  for (int mode = 0; mode < 4; ++mode) {
    Session s;
    auto i = ready();
    if (mode == 0) i.safe = false;
    if (mode == 1) i.fresh_request = false;
    if (mode == 2) i.normal_ready = false;
    if (mode == 3) i.id = 0;
    s.update(i);
    CHECK(s.phase == Phase::idle && s.normal_tx() && !s.diagnostic_tx());
  }
  // Both acknowledgements AND observed ELM327 configuration are necessary.
  for (int mode = 0; mode < 3; ++mode) {
    Session s;
    auto i = ready();
    s.update(i);
    i.now += 100000000;
    i.acknowledged = mode != 0;
    i.elm_ready = mode != 1;
    s.update(i);
    CHECK(s.phase == (mode == 2 ? Phase::scanning : Phase::preparing));
    CHECK(!s.normal_tx());
  }
  // Every loss of authority stops diagnostic TX and requires recovery.
  for (int mode = 0; mode < 7; ++mode) {
    Session s;
    auto i = ready();
    s.update(i);
    i.now += 100000000;
    i.acknowledged = i.elm_ready = true;
    s.update(i);
    CHECK(s.diagnostic_tx());
    if (mode == 0) i.safe = false;
    if (mode == 1) i.fresh_request = false;
    if (mode == 2) i.id++;
    if (mode == 3) i.active = false;
    if (mode == 4) i.acknowledged = false;
    if (mode == 5) i.elm_ready = false;
    if (mode == 6) i.now += 661000000000ULL;
    s.update(i);
    CHECK(s.phase == Phase::restoring && !s.normal_tx() && !s.diagnostic_tx());
  }
  {
    Session s;
    auto i = ready();
    s.update(i);
    i.acknowledged = i.elm_ready = true;
    s.update(i);
    i.route++;
    i.obd = false;
    s.update(i);
    CHECK(s.phase == Phase::preparing && !s.diagnostic_tx());
    i.acknowledged = i.elm_ready = false;
    s.update(i);
    CHECK(s.phase == Phase::preparing);
    i.now += 6000000000ULL;
    s.update(i);
    CHECK(s.phase == Phase::restoring);
    // A good old safety model alone cannot release the lock before a cycle.
    i.acknowledged = i.normal_ready = true;
    s.update(i);
    CHECK(s.phase == Phase::restoring);
    i.onroad = false;
    s.update(i);
    CHECK(!s.normal_tx());  // Missing deviceState is not an observed offroad transition.
    i.observed_offroad = true;
    s.update(i);
    CHECK(s.phase == Phase::restoring && s.normal_tx());
    i.onroad = true;
    i.observed_offroad = false;
    i.normal_ready = false;
    s.update(i);
    CHECK(s.phase == Phase::restoring);
    i.normal_ready = true;
    s.update(i);
    CHECK(s.phase == Phase::idle && s.normal_tx());
    s.update(i);
    CHECK(s.phase == Phase::idle);  // Stale/retried ID cannot restart a scan.
  }
  {
    Session s;
    s.recover(1, "process restarted");
    auto i = ready();
    s.update(i);
    CHECK(s.phase == Phase::restoring && !s.normal_tx());
  }
  {
    uint8_t read[] = {1, 3, 0, 0, 0, 0, 0, 0};
    uint8_t clear[] = {1, 4, 0, 0, 0, 0, 0, 0};
    uint8_t uds_clear[] = {4, 0x14, 0xff, 0xff, 0xff, 0, 0, 0};
    uint8_t reset[] = {2, 0x11, 1, 0, 0, 0, 0, 0};
    uint8_t write[] = {4, 0x2e, 0xf1, 0x90, 0, 0, 0, 0};
    uint8_t unlock[] = {2, 0x27, 1, 0, 0, 0, 0, 0};
    uint8_t extended[] = {0xf, 3, 0x19, 2, 0xff, 0, 0, 0};
    uint8_t flow[] = {0x30, 0, 0, 0, 0, 0, 0, 0};
    uint8_t functional[] = {2, 1, 0, 0, 0, 0, 0, 0};
    CHECK(diagnostic::frame(0x7e0, read, 8, 1));
    CHECK(diagnostic::frame(0x18da10f1, read, 8, 1));
    CHECK(diagnostic::frame(0x750, extended, 8, 0));
    CHECK(diagnostic::frame(0x7e0, flow, 8, 1));
    CHECK(!diagnostic::frame(0x7e0, read, 7, 1));
    CHECK(!diagnostic::frame(0x123, read, 8, 1));
    CHECK(!diagnostic::frame(0x7e0, read, 8, 4));
    CHECK(diagnostic::frame(0x7df, functional, 8, 1));
    CHECK(!diagnostic::frame(0x7df, read, 8, 1));
    for (auto data : {clear, uds_clear, reset, write, unlock}) CHECK(!diagnostic::frame(0x7e0, data, 8, 1));
  }
  std::cout << "diagnostic session policy tests passed\n";
}
