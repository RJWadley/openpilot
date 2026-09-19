#!/usr/bin/env python3
"""Rebuild the complete offline OBDex entries. Requires PyYAML only when updating the data."""
import gzip
import hashlib
import io
import json
import re
from pathlib import Path
from urllib.request import urlopen

import yaml


REVISION = "bc58b0eb7273226a1aabae98e956b70b8362bda1"
REPOSITORY = "https://github.com/foerbsnavi/OBDex"
FAMILIES = ("B0", "C0", "P0", "P2", "P3", "U0", "U3")
# Hashes retained from the original title-only dataset at REVISION.
SOURCE_SHA256 = {
  "data/generic/B0xxx_enriched.yaml": "73d317fb49b01a369ee29126bb0bb4f31c4775a5fe0a7e7fae1456aa980d3bc4",
  "data/generic/C0xxx_enriched.yaml": "1d60a394ff9cfde96b6e7a738420ac05dd48b85ebfe798001c38310aba7073e6",
  "data/generic/P0xxx_enriched.yaml": "a765cade770ffe756a5d4ea91c61fc128d5508f06f38552cd803dd726fecef63",
  "data/generic/P2xxx_enriched.yaml": "eda26317419f7e897c13eb254f74a01d905f180c7006174298d08d665b42dc4a",
  "data/generic/P3xxx_enriched.yaml": "bba19fe7dddb757866632ae939e4ace7dbbbe639816d3a46ce3a6936053708a3",
  "data/generic/U0xxx_enriched.yaml": "60aebde267b4bae53902e0abfeb7b489e314cd9e2ecdf0eb62384ea6b5a56afc",
  "data/generic/U3xxx_enriched.yaml": "f8a4b3723140b059216bad1d5fd450ce067c9c6bb8361f4120bbd422ecc769b7",
}


def build_dataset(sources):
  entries = {}
  hashes = {}
  for family in FAMILIES:
    name = f"data/generic/{family}xxx_enriched.yaml"
    raw = sources[name]
    hashes[name] = hashlib.sha256(raw).hexdigest()
    if hashes[name] != SOURCE_SHA256[name]:
      raise ValueError(f"Pinned source hash mismatch: {name}")
    document = yaml.safe_load(raw)
    if not isinstance(document, list):
      raise ValueError(f"Expected a list of entries: {name}")
    for entry in document:
      if not isinstance(entry, dict):
        raise ValueError(f"Invalid entry: {entry!r}")
      code, title = entry.get("code"), entry.get("title")
      if (not isinstance(code, str) or not re.fullmatch(r"[PCBU][0-3][0-9A-F]{3}", code) or
          not isinstance(title, dict) or not isinstance(title.get("en"), str) or not title["en"].strip()):
        raise ValueError(f"Invalid code/title: {code!r}")
      if code in entries:
        raise ValueError(f"Duplicate code: {code}")
      entries[code] = entry

  return {"source": REPOSITORY, "revision": REVISION, "license": "CC0-1.0", "source_sha256": hashes,
          "entries": dict(sorted(entries.items()))}


def encode_dataset(dataset):
  raw = (json.dumps(dataset, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
  output = io.BytesIO()
  # GzipFile avoids platform-dependent OS bytes from gzip.compress(mtime=0) on Python 3.12.
  with gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0, compresslevel=9) as stream:
    stream.write(raw)
  return output.getvalue()


def main():
  sources = {}
  for family in FAMILIES:
    name = f"data/generic/{family}xxx_enriched.yaml"
    with urlopen(f"https://raw.githubusercontent.com/foerbsnavi/OBDex/{REVISION}/{name}", timeout=30) as response:
      sources[name] = response.read()

  dataset = build_dataset(sources)
  compressed = encode_dataset(dataset)
  if json.loads(gzip.decompress(compressed)) != dataset:
    raise ValueError("Compressed JSON did not preserve the complete dataset")
  destination = Path(__file__).with_name("obdex.json.gz")
  destination.write_bytes(compressed)
  print(f"Wrote {len(dataset['entries'])} complete entries ({len(compressed):,} compressed bytes) to {destination}")


if __name__ == "__main__":
  main()
