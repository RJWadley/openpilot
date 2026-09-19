#!/usr/bin/env python3
"""Rebuild the offline label table. Requires PyYAML only when updating the data."""
import hashlib
import json
import re
from pathlib import Path
from urllib.request import urlopen

import yaml


REVISION = "bc58b0eb7273226a1aabae98e956b70b8362bda1"
REPOSITORY = "https://github.com/foerbsnavi/OBDex"
FAMILIES = ("B0", "C0", "P0", "P2", "P3", "U0", "U3")


def main():
  labels = {}
  hashes = {}
  for family in FAMILIES:
    name = f"data/generic/{family}xxx_enriched.yaml"
    with urlopen(f"https://raw.githubusercontent.com/foerbsnavi/OBDex/{REVISION}/{name}", timeout=30) as response:
      raw = response.read()
    hashes[name] = hashlib.sha256(raw).hexdigest()
    for entry in yaml.safe_load(raw):
      code, title = entry["code"], entry["title"]["en"]
      if not re.fullmatch(r"[PCBU][0-3][0-9A-F]{3}", code) or not isinstance(title, str) or not title.strip():
        raise ValueError(f"Invalid label: {code!r}")
      if code in labels:
        raise ValueError(f"Duplicate code: {code}")
      labels[code] = title.strip()

  dataset = {"source": REPOSITORY, "revision": REVISION, "license": "CC0-1.0", "source_sha256": hashes, "labels": dict(sorted(labels.items()))}
  destination = Path(__file__).with_name("obdex.json")
  destination.write_text(json.dumps(dataset, ensure_ascii=False, indent=2) + "\n")
  print(f"Wrote {len(labels)} labels to {destination}")


if __name__ == "__main__":
  main()
