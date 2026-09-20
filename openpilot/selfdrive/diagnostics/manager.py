"""One scan at a time, bounded persistent evidence, and a shared CLI/MCP engine."""
import fcntl
import json
import os
import re
import tempfile
import threading
import uuid
from pathlib import Path
from datetime import datetime, UTC

from tools.scripts.car import diagnose


def storage_root():
  return diagnose.module_cache_path().parent / "reports"


class ReportStore:
  def __init__(self, root=None, count=20, byte_limit=100 * 1024 * 1024):
    self.root = Path(root) if root is not None else storage_root()
    self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
    self.count, self.byte_limit = count, byte_limit
    self.lock = threading.Lock()

  def _files(self):
    return sorted((p for p in self.root.iterdir() if re.fullmatch(r"[0-9a-f]{32}\.json", p.name) and not p.is_symlink()),
                  key=lambda p: p.stat().st_mtime_ns, reverse=True)

  def save(self, scan_id, evidence):
    if not re.fullmatch(r"[0-9a-f]{32}", scan_id):
      raise ValueError("Invalid scan ID")
    report = diagnose.diagnosis_report(evidence)
    report['scan_id'] = scan_id
    data = json.dumps({"report": report, "evidence": evidence}, allow_nan=False).encode()
    if len(data) > self.byte_limit:
      raise ValueError("Report exceeds the local evidence storage limit")
    with self.lock:
      with tempfile.NamedTemporaryFile(dir=self.root, prefix=".scan-", delete=False) as stream:
        tmp = Path(stream.name)
        try:
          stream.write(data)
          stream.flush()
          os.fsync(stream.fileno())
          os.link(tmp, self.root / f"{scan_id}.json")  # exclusive final name
        finally:
          tmp.unlink(missing_ok=True)
      total = 0
      for i, path in enumerate(self._files()):
        total += path.stat().st_size
        if i >= self.count or total > self.byte_limit:
          path.unlink()
    return report

  def get(self, scan_id="latest", ecu=None):
    with self.lock:
      if scan_id == "latest":
        files = self._files()
        if not files:
          raise ValueError("No saved scans yet")
        path = files[0]
      elif re.fullmatch(r"[0-9a-f]{32}", scan_id):
        path = self.root / f"{scan_id}.json"
      else:
        raise ValueError("Invalid scan ID")
      if path.is_symlink():
        raise ValueError("Invalid report path")
      data = json.loads(path.read_text())
    if ecu is not None:
      address = int(ecu, 0)
      data['evidence']['ecus'] = [e for e in data['evidence']['ecus'] if int(e['tx_address'], 16) == address]
      # Discovery dumps are not relevant to a targeted evidence read.
      data['evidence'].pop('discovery', None)
    return data


def scan_args(target=None, broad=False, fast=False, details=False):
  if any(type(v) is not bool for v in (broad, fast, details)):
    raise ValueError("broad, fast and details must be booleans")
  args = diagnose.make_parser().parse_args([])
  args.broad, args.fast, args.details = broad, fast, details
  if target is not None:
    if not isinstance(target, str):
      raise ValueError("target must be a CAN address string, for example 0x715")
    args.addr = int(target, 0)
    if not diagnose.valid_tx(args.addr):
      raise ValueError("Target is not a permitted physical diagnostic address")
  return args


class DiagnosticManager:
  def __init__(self, store=None, transport_factory=None):
    self.store = store or ReportStore()
    if transport_factory is None:
      from openpilot.selfdrive.diagnostics.transport import MessagingPanda
      transport_factory = MessagingPanda
    self.transport_factory = transport_factory
    self.lock = threading.Lock()

  def scan(self, args, cancel=None):
    if not self.lock.acquire(blocking=False):
      raise RuntimeError("A diagnostic scan is already running")
    try:
      # Also exclude other CLI/server processes; never steal their pub sockets.
      with (self.store.root.parent / 'scan.lock').open('a') as lockfile:
        try:
          fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as e:
          raise RuntimeError("Another diagnostic client is scanning") from e
        return self._scan(args, cancel)
    finally:
      self.lock.release()

  def _scan(self, args, cancel):
    from opendbc.car.structs import CarParams
    scan_id = uuid.uuid4().hex
    warnings = []
    try:
      dataset = diagnose.load_dataset()
    except (OSError, ValueError, EOFError) as e:
      dataset = {}
      warnings.append(f"Offline descriptions unavailable: {e}")
    try:
      targets = diagnose.load_known_targets()
    except (ImportError, OSError) as e:
      targets = {}
      warnings.append(f"Brand hints unavailable: {e}")
    transport = None
    evidence = {"schema_version": diagnose.SCHEMA_VERSION, "report_kind": "technical_evidence", "status": "failed", "setup_error": True,
                "started_at": datetime.now(UTC).isoformat(), "vehicle_coverage_complete": False, "ecus": [], "errors": []}
    try:
      transport = self.transport_factory(cancel=cancel)
      evidence = diagnose.scan(transport, args, dataset, targets, CarParams.SafetyModel)
    except Exception as e:
      evidence['errors'].append(f"{type(e).__name__}: {e}")
    finally:
      if transport is not None:
        try:
          transport.close()
        except Exception as e:
          evidence['errors'].append(f"Recovery incomplete: {e}")
          evidence['recovery_required'] = True
    evidence.setdefault('warnings', []).extend(warnings)
    evidence['scan_id'] = scan_id
    report = self.store.save(scan_id, evidence)
    if evidence.get('recovery_required'):
      report['recovery_required'] = True
    return report
