"""One scan at a time, bounded persistent evidence, and a shared CLI/MCP engine."""
import fcntl
import json
import os
import re
import tempfile
import threading
import time
import uuid
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from datetime import datetime, UTC

from tools.scripts.car import diagnose
from openpilot.selfdrive.diagnostics.reports import compact_report, coverage_status, decode_cursor, evidence_report, paginate


def storage_root():
  return diagnose.module_cache_path().parent / "reports"


def timestamp():
  return datetime.now(UTC).isoformat()


class BusyError(RuntimeError):
  def __init__(self, status):
    super().__init__('A diagnostic scan is already running; inspect get_scan_status instead of retrying')
    self.status = status


@dataclass
class Operation:
  scan_id: str
  state: dict
  lockfile: object
  cancel: threading.Event = field(default_factory=threading.Event)
  done: threading.Event = field(default_factory=threading.Event)
  mutex: threading.Lock = field(default_factory=threading.Lock)
  result: dict | None = None
  last_saved: float = 0

  def snapshot(self):
    with self.mutex:
      return deepcopy(self.state)


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
      for view in ('report', 'evidence'):
        data[view]['ecus'] = [e for e in data[view]['ecus'] if int(e['tx_address'], 16) == address]
      # Discovery dumps are not relevant to a targeted evidence read.
      data['evidence'].pop('discovery', None)
    return data

  def get_report(self, scan_id="latest", ecu=None, cursor=None, limit=20):
    return self.get_evidence(scan_id, ecu, cursor, limit, raw=False)

  def get_evidence(self, scan_id="latest", ecu=None, cursor=None, limit=20, raw=False):
    if type(raw) is not bool:
      raise ValueError('raw must be a boolean')
    if cursor is not None:
      pinned_id = decode_cursor(cursor)['scan_id']
      if scan_id not in ('latest', pinned_id):
        raise ValueError('Cursor belongs to another scan')
      scan_id = pinned_id
    ecu = hex(int(ecu, 0)) if ecu is not None else None
    bundle = self.get(scan_id, ecu)
    report = evidence_report(bundle) if raw else compact_report(bundle['report'])
    active = self.operation_status()
    report.update(active_scan_id=active['active_scan_id'], is_active_scan=report['scan_id'] == active['active_scan_id'],
                  report_age_seconds=None)
    if report.get('started_at'):
      report['report_age_seconds'] = round(max(0, (datetime.now(UTC) - datetime.fromisoformat(report['started_at'])).total_seconds()), 1)
    return paginate(report, view='raw' if raw else 'compact', ecu=ecu, cursor=cursor, limit=limit)

  def save_status(self, state):
    scan_id = state['scan_id']
    if not re.fullmatch(r'[0-9a-f]{32}', scan_id):
      raise ValueError('Invalid scan ID')
    directory = self.root / 'operations'
    directory.mkdir(exist_ok=True, mode=0o700)
    with self.lock:
      with tempfile.NamedTemporaryFile(dir=directory, prefix='.status-', delete=False) as stream:
        tmp = Path(stream.name)
        try:
          stream.write(json.dumps(state, allow_nan=False).encode())
          stream.flush()
          os.fsync(stream.fileno())
          os.replace(tmp, directory / f'{scan_id}.json')
        finally:
          tmp.unlink(missing_ok=True)
      files = sorted((p for p in directory.iterdir() if re.fullmatch(r'[0-9a-f]{32}\.json', p.name) and not p.is_symlink()),
                     key=lambda p: p.stat().st_mtime_ns, reverse=True)
      for path in files[self.count:]:
        path.unlink()

  def read_status(self, scan_id='active'):
    directory = self.root / 'operations'
    with self.lock:
      if scan_id == 'active':
        files = sorted((p for p in directory.glob('*.json') if re.fullmatch(r'[0-9a-f]{32}\.json', p.name) and not p.is_symlink()),
                       key=lambda p: p.stat().st_mtime_ns, reverse=True)
        if not files:
          return None
        path = files[0]
      elif isinstance(scan_id, str) and re.fullmatch(r'[0-9a-f]{32}', scan_id):
        path = directory / f'{scan_id}.json'
      else:
        raise ValueError('Invalid scan ID')
      if path.is_symlink():
        raise ValueError('Invalid status path')
      return json.loads(path.read_text())

  def operation_status(self, scan_id='active'):
    if scan_id == 'latest':
      scan_id = self.get()['report']['scan_id']
    try:
      state = self.read_status(scan_id)
    except FileNotFoundError:
      report = self.get(scan_id)['report']
      return {'scan_id': scan_id, 'execution': report.get('execution', 'unknown'), 'phase': 'saved_report',
              'started_at': report.get('started_at'), 'updated_at': report.get('completed_at'), 'completed_at': report.get('completed_at'),
              'report_ready': True, 'coverage': coverage_status(report), 'vehicle_coverage_complete': False,
              'restoration': report.get('restoration', {'state': 'unknown'}), 'active_scan_id': None,
              'message': 'Saved report; older scans may not contain lifecycle or restoration evidence'}
    if state is None:
      return {'scan_id': None, 'execution': 'idle', 'phase': 'idle', 'active_scan_id': None}
    if state['execution'] == 'running':
      latest = self.read_status()
      with (self.root.parent / 'scan.lock').open('a') as lockfile:
        try:
          fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
          owner_present = False
        except BlockingIOError:
          owner_present = True
      if not owner_present or latest['scan_id'] != state['scan_id']:
        state.update(execution='interrupted', phase='interrupted', restoration={'state': 'unknown'},
                     message='Scan worker stopped; normal openpilot operation has not been verified')
    state['active_scan_id'] = state['scan_id'] if state['execution'] == 'running' else None
    state['seconds_since_update'] = round(max(0, (datetime.now(UTC) - datetime.fromisoformat(state['updated_at'])).total_seconds()), 1)
    return state


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
    self.operation = None

  def _reserve(self, cancel=None):
    if not self.lock.acquire(blocking=False):
      raise BusyError(self.get_status())
    lockfile = None
    try:
      lockfile = (self.store.root.parent / 'scan.lock').open('a')
      try:
        fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError as e:
        raise BusyError(self.get_status()) from e
      scan_id, now = uuid.uuid4().hex, timestamp()
      state = {'scan_id': scan_id, 'execution': 'running', 'phase': 'preflight', 'started_at': now, 'updated_at': now,
               'completed_at': None, 'collection_finished_at': None, 'report_ready': False, 'coverage': 'unknown',
               'vehicle_coverage_complete': False, 'restoration': {'state': 'not_started'},
               'progress': {'sequence': 0, 'message': 'Checking diagnostic prerequisites'}}
      operation = Operation(scan_id, state, lockfile, cancel=cancel or threading.Event())
      self.store.save_status(state)
      self.operation = operation
      return operation
    except BaseException:
      if lockfile is not None:
        lockfile.close()
      self.lock.release()
      raise

  def start(self, args):
    operation = self._reserve()
    try:
      threading.Thread(target=self._run, args=(operation, args), name='diagnostic-scan', daemon=True).start()
    except BaseException:
      operation.lockfile.close()
      self.lock.release()
      raise
    return operation

  def scan(self, args, cancel=None):
    operation = self._reserve(cancel)
    self._run(operation, args)
    if operation.result is None:
      raise RuntimeError(operation.snapshot().get('error', 'No diagnostic report was saved'))
    return operation.result

  def _update(self, operation, phase, message, progress_detail=None, **fields):
    with operation.mutex:
      old_phase = operation.state['phase']
      operation.state.update(phase=phase, updated_at=timestamp(), **fields)
      operation.state['progress'] = {'sequence': operation.state['progress']['sequence'] + 1, 'message': message, **(progress_detail or {})}
      snapshot = deepcopy(operation.state)
    if phase != old_phase or time.monotonic() - operation.last_saved >= 1:
      self.store.save_status(snapshot)
      operation.last_saved = time.monotonic()

  def get_status(self, scan_id='active'):
    if scan_id == 'latest':
      scan_id = self.store.get()['report']['scan_id']
    operation = self.operation
    if operation is not None and (scan_id == operation.scan_id or (scan_id == 'active' and not operation.done.is_set())):
      state = operation.snapshot()
    else:
      return self.store.operation_status(scan_id)
    state['active_scan_id'] = state['scan_id'] if state['execution'] == 'running' else None
    state['seconds_since_update'] = round(max(0, (datetime.now(UTC) - datetime.fromisoformat(state['updated_at'])).total_seconds()), 1)
    return state

  def _run(self, operation, args):
    try:
      self._scan(operation, args)
    except Exception as e:
      with operation.mutex:
        operation.state.update(execution='failed', phase='failed', error=f'{type(e).__name__}: {e}', completed_at=timestamp(),
                               updated_at=timestamp())
    finally:
      try:
        self.store.save_status(operation.snapshot())
      finally:
        operation.lockfile.close()
        self.lock.release()
        operation.done.set()

  def _scan(self, operation, args):
    from opendbc.car.structs import CarParams
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
                "started_at": operation.state['started_at'], "vehicle_coverage_complete": False, "ecus": [], "errors": []}
    try:
      transport = self.transport_factory(cancel=operation.cancel)
      def progress(event):
        self._update(operation, event['phase'], event['message'], progress_detail={k: v for k, v in event.items() if k not in ('phase', 'message')})
      evidence = diagnose.scan(transport, args, dataset, targets, CarParams.SafetyModel, progress=progress)
    except Exception as e:
      evidence['errors'].append(f"{type(e).__name__}: {e}")
    finally:
      # Close the transport even if persisting a progress update fails.
      try:
        self._update(operation, 'restoring', 'Data collection finished; restoring normal openpilot operation',
                     collection_finished_at=timestamp(), restoration={'state': 'in_progress'}, coverage=coverage_status(evidence))
      finally:
        if transport is not None:
          try:
            requested = getattr(transport, 'acquired', False) or getattr(transport, 'active', False)
            transport.close()
            restoration = {'state': 'verified' if requested else 'not_needed', 'checked_at': timestamp()}
          except Exception as e:
            evidence['errors'].append(f"Recovery incomplete: {e}")
            evidence['recovery_required'] = True
            restoration = {'state': 'unverified', 'error': str(e), 'checked_at': timestamp()}
        else:
          restoration = {'state': 'not_needed'}
        with operation.mutex:
          operation.state['restoration'] = restoration
    evidence.setdefault('warnings', []).extend(warnings)
    execution = 'cancelled' if operation.cancel.is_set() else 'failed' if evidence['status'] == 'failed' else 'finished'
    evidence.update(scan_id=operation.scan_id, execution=execution, completed_at=timestamp(), restoration=restoration,
                    collection_finished_at=operation.state['collection_finished_at'])
    operation.result = self.store.save(operation.scan_id, evidence)
    outcome = 'Scan complete' if execution == 'finished' else f'Scan {execution}'
    message = f'{outcome}; normal openpilot operation restored' if restoration['state'] == 'verified' else \
              f'{outcome}; normal openpilot operation not verified' if restoration['state'] == 'unverified' else f'{outcome}; no restoration needed'
    self._update(operation, 'complete' if execution == 'finished' else execution, message,
                 execution=execution, completed_at=evidence['completed_at'], report_ready=True,
                 coverage=coverage_status(evidence), restoration=restoration)
