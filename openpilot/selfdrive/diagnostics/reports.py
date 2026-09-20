"""Model-facing projections. Stored evidence is never modified by presentation."""
from copy import deepcopy
import base64
import json

PAGE_BYTES = 8000


def coverage_status(report):
  return {'failed': 'unavailable', 'complete': 'best_effort'}.get(report.get('status'), report.get('status', 'unknown'))


def decode_cursor(cursor):
  try:
    if not isinstance(cursor, str) or len(cursor) > 1024:
      raise ValueError
    value = json.loads(base64.urlsafe_b64decode(cursor.encode()))
    if not isinstance(value, dict) or set(value) != {'scan_id', 'view', 'ecu', 'index', 'offset'}:
      raise ValueError
    if any(type(value[key]) is not int or value[key] < 0 for key in ('index', 'offset')):
      raise ValueError
    return value
  except (ValueError, TypeError, UnicodeError) as e:
    raise ValueError('Invalid page cursor') from e


def paginate(report, *, view, ecu=None, cursor=None, limit=20):
  """Bound bytes as well as item count; an oversized item is losslessly fragmented."""
  if type(limit) is not int or not 1 <= limit <= 50:
    raise ValueError('limit must be an integer from 1 to 50')
  binding = {'scan_id': report['scan_id'], 'view': view, 'ecu': ecu}
  index, offset = 0, 0
  if cursor is not None:
    position = decode_cursor(cursor)
    if any(position[key] != value for key, value in binding.items()):
      raise ValueError('Cursor belongs to a different scan, view, or ECU filter')
    index, offset = position['index'], position['offset']
  items = report.pop('items')
  # Even unexpected large metadata must obey the byte bound.
  dynamic_keys = ('report_age_seconds', 'active_scan_id', 'is_active_scan')
  stable = {key: value for key, value in report.items() if key not in dynamic_keys}
  if len(json.dumps(stable).encode()) > 4000:
    items = [{'kind': 'metadata', 'value': stable}] + items
    report = {key: report[key] for key in ('scan_id', 'server_version', 'interpretation', 'reference_notice', *dynamic_keys) if key in report}
  if index > len(items) or (index == len(items) and offset):
    raise ValueError('Cursor is outside this report')
  page = {**report, 'view': view, 'ecu_filter': ecu, 'summary_scope': 'entire_scan', 'items': [], 'next_cursor': None}

  def continuation(i, o):
    return base64.urlsafe_b64encode(json.dumps({**binding, 'index': i, 'offset': o}).encode()).decode() if i < len(items) else None

  def fits(item, i, o):
    candidate = {**page, 'items': [*page['items'], item], 'next_cursor': continuation(i, o)}
    return len(json.dumps(candidate).encode()) <= PAGE_BYTES

  while index < len(items) and len(page['items']) < limit:
    item = items[index]
    if not offset and fits(item, index + 1, 0):
      page['items'].append(item)
      index += 1
      continue
    if page['items']:
      break
    serialized = json.dumps(item)
    if offset >= len(serialized):
      raise ValueError('Cursor is outside this record')
    # JSON escaping can expand text by 6x; measure the actual envelope.
    low, high = 1, len(serialized) - offset
    best = None
    while low <= high:
      length = (low + high) // 2
      final = offset + length == len(serialized)
      fragment = {'kind': 'json_fragment', 'record_index': index, 'offset': offset, 'total_chars': len(serialized),
                  'text': serialized[offset:offset + length], 'final': final}
      next_index, next_offset = (index + 1, 0) if final else (index, offset + length)
      if fits(fragment, next_index, next_offset):
        best = fragment, next_index, next_offset
        low = length + 1
      else:
        high = length - 1
    if best is None:
      raise ValueError('Report metadata exceeds page limit')
    fragment, index, offset = best
    page['items'].append(fragment)
    break
  page['next_cursor'] = continuation(index, offset)
  return page


def evidence_report(bundle):
  report = compact_report(bundle['report'])
  report['items'] = []
  for name, value in bundle['evidence'].items():
    if name in ('ecus', 'discovery'):
      if not value:
        report['items'].append({'kind': 'evidence', 'path': [name], 'value': value})
      for i, entry in enumerate(value):
        if not entry:
          report['items'].append({'kind': 'evidence', 'path': [name, i], 'value': entry})
        for key, field in entry.items():
          if isinstance(field, list):
            report['items'].extend({'kind': 'evidence', 'path': [name, i, key, j], 'value': child} for j, child in enumerate(field))
            if not field:
              report['items'].append({'kind': 'evidence', 'path': [name, i, key], 'value': []})
          else:
            report['items'].append({'kind': 'evidence', 'path': [name, i, key], 'value': field})
    else:
      report['items'].append({'kind': 'evidence', 'path': [name], 'value': value})
  return report


def compact_report(report):
  result = {key: report[key] for key in ('scan_id', 'server_version', 'started_at', 'completed_at', 'execution', 'restoration',
                                        'coverage', 'coverage_gaps', 'vehicle_coverage_complete', 'scope', 'summary',
                                        'interpretation', 'reference_notice') if key in report}
  result.update(execution=report.get('execution', 'unknown'), restoration=report.get('restoration', {'state': 'unknown'}),
                coverage=coverage_status(report), coverage_method=report.get('coverage', 'unknown'))
  result['items'] = []
  # Put actionable limitations before findings so they are not buried on the
  # last page. Counts and coverage gaps stay in every compact page's metadata.
  for kind in ('errors', 'warnings'):
    result['items'].extend({'kind': kind[:-1], 'message': message} for message in report.get(kind, []))
  for ecu in report['ecus']:
    identity = {key: ecu[key] for key in ('bus', 'obd_multiplexing', 'tx_address', 'rx_address', 'subaddress', 'identity') if key in ecu}
    result['items'].append({'kind': 'ecu', **{key: value for key, value in ecu.items() if key != 'codes'}})
    for code in ecu['codes']:
      fault = deepcopy(code)
      if 'lookup' in fault:
        fault['lookup'].pop('entry', None)
        fault['lookup']['details_available'] = True
      result['items'].append({'kind': 'fault', 'ecu': identity, 'fault': fault})
  return result
