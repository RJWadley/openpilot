"""One safe inventory preparation attempt per device boot; faults stay on demand."""
import json
import logging
import re
import threading
import time
from pathlib import Path

from tools.scripts.car import diagnose
from openpilot.selfdrive.diagnostics.manager import BusyError


class StartupPreparation:
  def __init__(self, manager, boot_id, marker=None):
    if re.fullmatch(r'[0-9a-f-]{36}', boot_id) is None:
      raise ValueError('A Linux boot ID is required for startup preparation')
    self.manager, self.boot_id = manager, boot_id
    self.marker = Path(marker) if marker is not None else manager.store.root.parent / 'preparation-boot.json'
    self.stop = threading.Event()
    self.thread = None
    self.ready_since = None
    try:
      self.attempted = json.loads(self.marker.read_text()).get('boot_id') == boot_id
    except (OSError, ValueError, AttributeError):
      self.attempted = False

  def tick(self, state, *, valid, stamp, now):
    """Readiness is advisory: pandad rechecks every safety gate when acquiring TX."""
    if self.attempted or self.stop.is_set():
      return
    # An explicit scan in this process already performs preparation as needed.
    requested = self.manager.operation is not None
    ready = valid and stamp > 0 and 0 <= now - stamp < 1 and str(state.phase) == 'idle' and state.ready
    if not requested:
      if not ready:
        self.ready_since = None
        return
      if self.ready_since is None:
        self.ready_since = now
      if now - self.ready_since < 2:
        return
    # Persist BEFORE starting. Recovery's onroad cycle and diagnosticd restarts
    # must not schedule another automatic attempt in the same actual OS boot.
    diagnose.save_module_cache(self.marker, {'boot_id': self.boot_id})
    self.attempted = True
    if not requested and not self.stop.is_set():
      try:
        self.manager.prepare()
      except BusyError:
        pass  # A concurrent CLI/MCP operation owns preparation; never queue a retry.

  def start(self):
    self.thread = threading.Thread(target=self._run, name='diagnostic-startup', daemon=True)
    self.thread.start()
    return self

  def _run(self):
    try:
      from openpilot.cereal import messaging
      sm = messaging.SubMaster(['diagnosticState'])
      while not self.stop.wait(0.2) and not self.attempted:
        sm.update(0)
        self.tick(sm['diagnosticState'], valid=sm.valid['diagnosticState'],
                  stamp=sm.logMonoTime['diagnosticState'] / 1e9, now=time.monotonic())
    except Exception:
      # Preparation failure must not take down the MCP or driving processes.
      logging.exception('Automatic diagnostic preparation unavailable; explicit scans remain available')

  def close(self):
    self.stop.set()
    if self.thread is not None:
      self.thread.join(timeout=2)


def start_preparation(manager):
  if not Path('/AGNOS').is_file():
    return None  # Local development must never automatically contact a vehicle.
  try:
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    return StartupPreparation(manager, boot_id).start()
  except (OSError, ValueError, RuntimeError):
    logging.exception('Could not start automatic diagnostics preparation; explicit scans remain available')
    return None
