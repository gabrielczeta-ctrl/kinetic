"""Summarize discrete read-only posture queries, not walking acceptance."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re


def summarize(path):
    path = Path(path)
    steps = {}
    for line in path.read_text().splitlines():
        if not line.startswith("COORDINATED_QUERY "):
            continue
        f = dict(re.findall(r"(\w+)=([^ ]+)", line))
        if f.get("schema") != "2":
            raise ValueError("Expected explicit query schema 2")
        step, fraction = int(f["step"]), float(f["heading_fraction"])
        advance = float(f["heading_advance"])
        shift = [float(x) for x in f["shift"].split(",")]
        if not 0 <= fraction <= 1 or len(shift) != 3 or not all(map(math.isfinite, [advance, *shift])):
            raise ValueError("Invalid query numbers")
        if f["valid"] not in ("0", "1") or f["applied"] not in ("0", "1"):
            raise ValueError("Invalid query flags")
        rows = steps.setdefault(step, {})
        if fraction in rows:
            raise ValueError("Duplicate step/fraction; select one replay")
        rows[fraction] = {"fraction": fraction, "advance_degrees": math.degrees(advance),
                          "pose_found": f["applied"] == "1", "pelvis_shift": shift}
    if not steps:
        raise ValueError("No query records")
    return {"source": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "scope": "Discrete geometry samples only; no continuous feasible interval or dynamic acceptance claim",
            "steps": [{"step": step, "queries": [rows[f] for f in sorted(rows, reverse=True)]}
                      for step, rows in sorted(steps.items())]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    args = parser.parse_args()
    print(json.dumps(summarize(args.log), indent=2))
