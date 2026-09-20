"""Real HTTP + msgq + ISO-TP, with simulated coordinator and ECUs (no hardware)."""
import gc
import http.client
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from openpilot.cereal import messaging
from openpilot.selfdrive.diagnostics.manager import DiagnosticManager, ReportStore
from openpilot.selfdrive.diagnostics.mcp import MCPServer
from openpilot.selfdrive.diagnostics.transport import MessagingPanda
from tools.scripts.car import diagnose
from tools.scripts.car.tests.test_diagnose import FakePanda


class SimulatedCoordinator:
  def __init__(self):
    self.stop = threading.Event()
    self.ready = threading.Event()
    self.failure = None
    self.abort = False
    self.stale = False
    self.routes = []
    self.recovered = False
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()
    assert self.ready.wait(2)

  def run(self):
    try:
      pm = messaging.PubMaster(['diagnosticState', 'pandaStates', 'can'])
      sm = messaging.SubMaster(['diagnosticRequest'])
      tx = messaging.sub_sock('diagnosticSendcan')
      target = diagnose.Target(0x7e0, 0x7e8)
      panda = FakePanda({target: {b'\x03': bytes.fromhex('43010202')}})
      active_id, route, phase, obd = 0, 0, 'idle', True
      restore_until = 0
      self.ready.set()
      while not self.stop.wait(0.005):
        sm.update(0)
        if sm.updated['diagnosticRequest'] and sm.valid['diagnosticRequest']:
          req = sm['diagnosticRequest']
          if req.active and not self.abort:
            active_id, route, phase, obd = req.sessionId, req.route, 'scanning', req.obd
            if not self.routes or self.routes[-1] != (route, obd):
              self.routes.append((route, obd))
          elif active_id and phase == 'scanning':
            phase, restore_until = 'restoring', time.monotonic() + 0.1
        if self.abort and phase == 'scanning':
          phase, restore_until = 'restoring', time.monotonic() + 0.1
        if phase == 'restoring' and time.monotonic() > restore_until:
          phase = 'idle'
          self.recovered = True
        if not self.stale:
          state = messaging.new_message('diagnosticState', valid=True)
          state.diagnosticState = {'sessionId': active_id, 'route': route, 'obd': obd, 'phase': phase,
                                   'error': 'state unsafe' if self.abort else ''}
          pm.send('diagnosticState', state)
        health = messaging.new_message('pandaStates', 1, valid=True)
        health.pandaStates[0] = {'ignitionLine': True, 'harnessStatus': 'normal', 'voltage': 12000, 'faults': [],
                                'pandaType': 'tres', 'canState1': {'lastError': 'ackError'}}
        pm.send('pandaStates', health)
        for packet in messaging.drain_sock(tx):
          request = packet.diagnosticSendcan
          if packet.valid and phase == 'scanning' and request.sessionId == active_id and request.route == route:
            for frame in request.frames:
              panda.can_send(frame.address, bytes(frame.dat), frame.src)
        replies = panda.can_recv()
        if replies:
          packet = messaging.new_message('can', len(replies), valid=True)
          for i, (addr, data, bus) in enumerate(replies):
            packet.can[i] = {'address': addr, 'dat': data, 'src': bus}
          pm.send('can', packet)
    except BaseException as e:
      self.failure = e
      self.ready.set()

  def close(self):
    self.stop.set()
    self.thread.join(timeout=2)


class TestTransport(unittest.TestCase):
  def setUp(self):
    root = '/tmp' if sys.platform == 'darwin' else '/dev/shm'
    self.queue_dir = tempfile.TemporaryDirectory(prefix='msgq_diagnostics-', dir=root)
    prefix = Path(self.queue_dir.name).name.removeprefix('msgq_')
    self.env = patch.dict(os.environ, {'OPENPILOT_PREFIX': prefix})
    self.env.start()
    self.sim = SimulatedCoordinator()

  def tearDown(self):
    self.sim.close()
    self.env.stop()
    gc.collect()
    self.queue_dir.cleanup()
    if self.sim.failure:
      raise self.sim.failure

  def test_routes_health_and_local_queue(self):
    from opendbc.car.structs import CarParams
    panda = MessagingPanda()
    try:
      self.assertEqual(panda.health()['car_harness_status'], 1)
      self.assertEqual(panda.can_health(1)['last_error'], 'AckError')
      panda.set_safety_mode(CarParams.SafetyModel.elm327, 0)
      panda.can_clear(0xffff)
      panda.set_safety_mode(CarParams.SafetyModel.elm327, 1)
      self.assertEqual(self.sim.routes[-1], (2, False))
      with self.assertRaises(ValueError):
        panda.set_safety_mode(CarParams.SafetyModel.allOutput)
      with self.assertRaises(ValueError):
        panda.can_clear(1)
    finally:
      panda.close()
    self.assertTrue(self.sim.recovered)
    self.assertFalse(panda.thread.is_alive())

  def test_native_abort_prevents_further_tx(self):
    from opendbc.car.structs import CarParams
    panda = MessagingPanda()
    try:
      panda.set_safety_mode(CarParams.SafetyModel.elm327)
      self.sim.abort = True
      deadline = time.monotonic() + 1
      while time.monotonic() < deadline and not self.sim.recovered:
        time.sleep(0.01)
      with self.assertRaisesRegex(RuntimeError, 'state unsafe'):
        panda.can_send(0x7e0, diagnose.single_frame(b'\x03'), 1)
    finally:
      panda.close()

  def test_cancellation_still_allows_default_session_cleanup(self):
    from opendbc.car.structs import CarParams
    cancel = threading.Event()
    panda = MessagingPanda(cancel)
    try:
      panda.set_safety_mode(CarParams.SafetyModel.elm327)
      cancel.set()
      with self.assertRaisesRegex(RuntimeError, 'cancelled'):
        panda.can_send(0x7e0, diagnose.single_frame(b'\x03'), 1)
      panda.can_send(0x7e0, diagnose.single_frame(b'\x10\x01'), 1)
    finally:
      panda.close()

  def test_http_to_real_scan_engine_and_saved_obdex_report(self):
    with tempfile.TemporaryDirectory() as root:
      manager = DiagnosticManager(ReportStore(root))
      server = MCPServer(('127.0.0.1', 0), manager)
      thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
      thread.start()
      headers = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}
      def request(body):
        conn = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=15)
        conn.request('POST', '/mcp', json.dumps(body), headers)
        response = conn.getresponse()
        data = response.read().decode()
        sid = response.getheader('MCP-Session-Id')
        conn.close()
        return data, sid
      try:
        _, sid = request({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {'protocolVersion': '2025-11-25'}})
        headers.update({'MCP-Session-Id': sid, 'MCP-Protocol-Version': '2025-11-25'})
        request({'jsonrpc': '2.0', 'method': 'notifications/initialized'})
        with patch.object(diagnose, 'load_known_targets', return_value={}):
          data, _ = request({'jsonrpc': '2.0', 'id': 2, 'method': 'tools/call',
                             'params': {'name': 'scan_vehicle', 'arguments': {'target': '0x7e0', 'wait': True},
                                        '_meta': {'progressToken': 'scan-progress'}}})
        responses = [json.loads(line[6:]) for line in data.splitlines() if line.startswith('data: ')]
        result = responses[-1]['result']
        self.assertFalse(result['isError'], result)
        report = result['structuredContent']
        codes = [i['fault'] for i in report['items'] if i['kind'] == 'fault']
        code = next(c for c in codes if c['code'] == 'P0202')
        self.assertTrue(code['lookup']['title'])
        self.assertNotIn('entry', code['lookup'])
        self.assertFalse(report['vehicle_coverage_complete'])
        self.assertEqual(report['restoration']['state'], 'verified')
        progress = [message['params'] for message in responses if message.get('method') == 'notifications/progress']
        self.assertTrue(progress)
        self.assertTrue(all(p['progressToken'] == 'scan-progress' for p in progress))
        self.assertTrue(all(b['progress'] > a['progress'] for a, b in zip(progress, progress[1:], strict=False)))
        self.assertIn('normal openpilot operation restored', progress[-1]['message'])
        self.assertTrue(self.sim.recovered)
        stored = manager.store.get(report['scan_id'])
        self.assertTrue(any(c.get('lookup', {}).get('entry', {}).get('code') == 'P0202' for e in stored['report']['ecus'] for c in e['codes']))
        self.assertTrue(stored['evidence']['ecus'][0]['queries'])
        data, _ = request({'jsonrpc': '2.0', 'id': 3, 'method': 'tools/call',
                           'params': {'name': 'get_scan_report', 'arguments': {'scan_id': report['scan_id'], 'ecu': '0x7e0'}}})
        page = json.loads(data)['result']['structuredContent']
        self.assertLessEqual(len(json.dumps(page).encode()), 8000)
        self.assertEqual(page['summary']['fault_history_records'], len(codes))
        raw_codes, cursor = [], None
        while True:
          arguments = {'scan_id': report['scan_id'], 'ecu': '0x7e0', 'raw': True}
          if cursor is not None:
            arguments['cursor'] = cursor
          data, _ = request({'jsonrpc': '2.0', 'id': 4, 'method': 'tools/call',
                             'params': {'name': 'get_scan_evidence', 'arguments': arguments}})
          page = json.loads(data)['result']['structuredContent']
          self.assertLessEqual(len(json.dumps(page).encode()), 8000)
          raw_codes.extend(i['value'] for i in page['items'] if i['kind'] == 'evidence' and i['path'][:3] == ['ecus', 0, 'codes'])
          cursor = page['next_cursor']
          if cursor is None:
            break
        self.assertTrue(any(c.get('lookup', {}).get('entry', {}).get('code') == 'P0202' for c in raw_codes))
      finally:
        server.cancel_all()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        for job in server.jobs.values():
          self.assertTrue(job.done.wait(5))
