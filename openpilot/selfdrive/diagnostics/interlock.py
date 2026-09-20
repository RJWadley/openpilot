"""Consumer-side latch: losing pandad must never release a diagnostic lockout."""
import time


class DiagnosticInterlock:
  def __init__(self, recovery_required=False):
    self.phase = "restoring" if recovery_required else "idle"
    self.session_id = 0
    self.route = 0
    self.last_update = time.monotonic_ns() if recovery_required else 0

  def update(self, sm):
    stamp = sm.logMonoTime['diagnosticState']
    fresh = 0 <= time.monotonic_ns() - stamp < 1_000_000_000 and stamp >= self.last_update
    if sm.updated['diagnosticState'] and sm.valid['diagnosticState'] and fresh:
      state = sm['diagnosticState']
      self.phase = str(state.phase)
      self.session_id, self.route = state.sessionId, state.route
      self.last_update = stamp

  @property
  def blocked(self):
    return self.phase != "idle"

  @property
  def pause_controls(self):
    return self.phase in ("preparing", "scanning")

  def acknowledge(self, pm, name, messaging):
    msg = messaging.new_message(name)
    msg.valid = True
    ack = getattr(msg, name)
    ack.sessionId, ack.route = self.session_id, self.route
    pm.send(name, msg)
