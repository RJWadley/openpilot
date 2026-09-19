import gzip
import hashlib
import json
import unittest
from unittest.mock import patch

import yaml

from tools.scripts.car.data import update_obdex as u


class TestUpdateObdex(unittest.TestCase):
  def build(self, documents):
    sources = {f"data/generic/{family}xxx_enriched.yaml": yaml.safe_dump(documents.get(family, []), allow_unicode=True).encode()
               for family in u.FAMILIES}
    hashes = {name: hashlib.sha256(raw).hexdigest() for name, raw in sources.items()}
    with patch.object(u, "SOURCE_SHA256", hashes):
      return u.build_dataset(sources)

  def test_complete_nested_entries_survive_gzip_json(self):
    entry = {"code": "P0301", "title": {"en": "  Cylinder 1 misfire  ", "de": "Zündaussetzer", "ja": "失火"},
             "description": {"en": "First line.\nSecond line.\n"},
             "common_causes": [{"id": "coil", "label": {"de": "Zündspule"}, "likelihood": "high"}],
             "repair": {"estimated_cost_eur": [80, 600], "estimated_hours": [0.5, 3], "diy_possible": True},
             "future_field": {"nested": [None, False, 0, "", {"values": []}]}, "references": ["sae:J2012"]}
    dataset = self.build({"P0": [entry]})
    decoded = json.loads(gzip.decompress(u.encode_dataset(dataset)))
    self.assertEqual(decoded["entries"], {"P0301": entry})
    self.assertEqual(decoded, dataset)
    self.assertNotIn("labels", decoded)
    self.assertEqual(len(decoded["source_sha256"]), 7)
    self.assertEqual(decoded["revision"], u.REVISION)
    self.assertEqual(decoded["source"], u.REPOSITORY)
    self.assertEqual(decoded["license"], "CC0-1.0")

  def test_output_is_deterministic_without_timestamp_or_filename(self):
    dataset = self.build({"B0": [{"code": "B0001", "title": {"en": "Airbag"}}]})
    first = u.encode_dataset(dataset)
    reordered = {key: dataset[key] for key in reversed(dataset)}
    self.assertEqual(first, u.encode_dataset(reordered))
    self.assertEqual(first[:3], b"\x1f\x8b\x08")
    self.assertEqual(first[3], 0)  # No optional header fields, including filename.
    self.assertEqual(first[4:8], b"\x00" * 4)

  def test_duplicate_codes_are_rejected_across_families(self):
    entry = {"code": "P0301", "title": {"en": "Misfire"}}
    with self.assertRaisesRegex(ValueError, "Duplicate code"):
      self.build({"P0": [entry], "P2": [entry]})

  def test_invalid_codes_and_titles_are_rejected(self):
    for entry in (None, {}, {"code": 301, "title": {"en": "Misfire"}}, {"code": "P030G", "title": {"en": "Misfire"}},
                  {"code": "P0301", "title": "Misfire"}, {"code": "P0301", "title": {"en": " "}}):
      with self.subTest(entry=entry), self.assertRaises(ValueError):
        self.build({"P0": [entry]})

  def test_changed_upstream_source_is_rejected(self):
    sources = dict.fromkeys(u.SOURCE_SHA256, b"[]\n")
    with self.assertRaisesRegex(ValueError, "Pinned source hash mismatch"):
      u.build_dataset(sources)


if __name__ == "__main__":
  unittest.main()
