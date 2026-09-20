"""Panda-shaped CAN adapter; pandad alone opens and configures the hardware."""
import secrets
import threading
import time


class MessagingPanda:
  def __init__(self, cancel=None):
    from openpilot.cereal import messaging
    self.messaging = messaging
    self.cancel = cancel or threading.Event()
    self.session_id = secrets.randbits(63) or 1
    self.route = 0
    self.obd = True
    self.active = False
    self.closed = False
    self.acquired = False
    self.lock = threading.Lock()
    self.stop = threading.Event()
    self.failure = None
    self.sm = messaging.SubMaster(['diagnosticState', 'pandaStates'])
    self.can_sock = messaging.sub_sock('can')
    self.tx = messaging.PubMaster(['diagnosticSendcan'])
    self.thread = threading.Thread(target=self._heartbeat, name="diagnostic-lease", daemon=True)
    self.thread.start()

  def _heartbeat(self):
    try:
      pm = self.messaging.PubMaster(['diagnosticRequest'])
      while not self.stop.is_set():
        msg = self.messaging.new_message('diagnosticRequest')
        msg.valid = True
        with self.lock:
          msg.diagnosticRequest = {"sessionId": self.session_id, "active": self.active, "route": self.route, "obd": self.obd}
        pm.send('diagnosticRequest', msg)
        self.stop.wait(0.1)
    except Exception as e:
      self.failure = str(e)

  def _state(self):
    self.sm.update(0)
    now = time.monotonic_ns()
    stamp = self.sm.logMonoTime['diagnosticState']
    if not self.sm.valid['diagnosticState'] or not 0 < now - stamp < 1_000_000_000:
      raise RuntimeError("pandad diagnostic coordinator unavailable; build and restart openpilot first")
    return self.sm['diagnosticState']

  def _check(self):
    if self.failure:
      raise RuntimeError(f"Diagnostic heartbeat failed: {self.failure}")
    state = self._state()
    if state.sessionId != self.session_id or str(state.phase) != "scanning" or state.route != self.route:
      raise RuntimeError(state.error or "Diagnostic session is not active")

  def set_safety_mode(self, mode, param=0):
    from opendbc.car.structs import CarParams
    if mode == CarParams.SafetyModel.noOutput:
      self.close()
      return
    if mode != CarParams.SafetyModel.elm327 or param not in (0, 1):
      raise ValueError("Diagnostics cannot select arbitrary Panda safety modes")
    if self.cancel.is_set():
      raise RuntimeError("Scan cancelled")
    with self.lock:
      self.route += 1
      self.obd = param == 0
      self.active = True
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
      try:
        state = self._state()
      except RuntimeError:
        time.sleep(0.02)
        continue
      if state.sessionId == self.session_id:
        if state.error or str(state.phase) == "restoring":
          raise RuntimeError(state.error or "Diagnostic session is recovering")
        if str(state.phase) == "scanning" and state.route == self.route and state.obd == self.obd:
          self.acquired = True
          return
      elif str(state.phase) != "idle":
        raise RuntimeError("Another diagnostic session is active or recovering")
      if self.cancel.is_set() or self.failure:
        raise RuntimeError(self.failure or "Scan cancelled")
      time.sleep(0.02)
    raise RuntimeError("Timed out waiting for diagnostic coordinator and engagement lockout")

  def can_send(self, address, data, bus, timeout=100):
    # Permit the reader's best-effort default-session cleanup after cancellation.
    cleanup = data[:3] == b'\x02\x10\x01' or data[1:4] == b'\x02\x10\x01'
    if self.cancel.is_set() and not cleanup:
      raise RuntimeError("Scan cancelled; retaining completed results")
    self._check()
    msg = self.messaging.new_message('diagnosticSendcan')
    msg.valid = True
    msg.diagnosticSendcan = {"sessionId": self.session_id, "route": self.route,
                             "frames": [{"address": address, "dat": data, "src": bus}]}
    self.tx.send('diagnosticSendcan', msg)

  def can_recv(self):
    self._check()
    return [(f.address, bytes(f.dat), f.src) for packet in self.messaging.drain_sock(self.can_sock) if packet.valid for f in packet.can]

  def can_clear(self, bus):
    if bus != 0xffff:
      raise ValueError("Only the scanner's local receive queue may be drained")
    self.messaging.drain_sock(self.can_sock)

  def _health(self):
    self.sm.update(0)
    stamp = self.sm.logMonoTime['pandaStates']
    if not self.sm.valid['pandaStates'] or not 0 < time.monotonic_ns() - stamp < 1_000_000_000 or len(self.sm['pandaStates']) != 1:
      raise RuntimeError("Fresh health from exactly one Panda is required")
    return self.sm['pandaStates'][0]

  def health(self):
    # Allow the first health publication to arrive before the scanner's preflight.
    deadline = time.monotonic() + 2
    while True:
      try:
        p = self._health()
        return {"ignition_line": p.ignitionLine, "ignition_can": p.ignitionCan, "car_harness_status": p.harnessStatus.raw,
                "voltage": p.voltage, "faults": sum(1 << f.raw for f in p.faults), "fault_status": p.faultStatus.raw}
      except RuntimeError:
        if time.monotonic() >= deadline:
          raise
        time.sleep(0.02)

  def can_health(self, bus):
    cs = getattr(self._health(), f"canState{bus}")
    return {"bus_off": cs.busOff, "total_error_cnt": cs.totalErrorCnt,
            "last_error": "AckError" if str(cs.lastError) == "ackError" else str(cs.lastError),
            "last_stored_error": "AckError" if str(cs.lastStoredError) == "ackError" else str(cs.lastStoredError)}

  def is_internal(self):
    return True

  def close(self):
    if self.closed:
      return
    self.closed = True
    with self.lock:
      self.active = False
    try:
      deadline = time.monotonic() + 60
      while time.monotonic() < deadline:
        state = self._state()
        if str(state.phase) == "idle" and (state.sessionId == self.session_id or not self.acquired):
          return
        time.sleep(0.1)
      raise RuntimeError("Recovery has not completed; engagement remains blocked. Check the device before driving with openpilot.")
    finally:
      self.stop.set()
      self.thread.join(timeout=2)
