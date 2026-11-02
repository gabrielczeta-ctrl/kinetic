"""Project planned landing onto the previous solved pelvis lateral axis.

This diagnoses geometry, not reachability or a fall cause. CSV commands are
pre-solve; body observations are post-solve, hence the one-row alignment.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def analyze(path, first, last):
    rows = list(csv.DictReader(Path(path).open(newline="")))
    samples = []
    for previous, row in zip(rows, rows[1:]):
        step = int(row["step"])
        if not first <= step <= last or not row["reference_placement_x"]:
            continue
        phase = int(row["gait_phase"])
        if phase not in (1, 2):
            continue
        side = 1 if phase == 1 else -1
        stance = "left" if phase == 1 else "right"
        heading = float(previous["pelvis_heading"])
        dx = float(row["reference_placement_x"]) - float(previous[stance + "_foot_x"])
        dy = float(row["reference_placement_y"]) - float(previous[stance + "_foot_y"])
        signed_width = side * (dx * math.cos(heading) + dy * math.sin(heading))
        samples.append({"step": step, "phase": phase,
                        "body_side_width": signed_width,
                        "previous_pelvis_up": float(previous["pelvis_up"])})
    crossed = [s for s in samples if s["body_side_width"] < 0]
    return {"source": str(Path(path).resolve()), "window": [first, last],
            "interpretation": "Negative means landing lies across stance foot in measured pelvis lateral frame; not proof of infeasibility or causation.",
            "samples": len(samples), "opposite_side_samples": len(crossed),
            "first_opposite_side": crossed[0] if crossed else None,
            "minimum": min(samples, key=lambda s: s["body_side_width"]) if samples else None,
            "rows": samples}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--first", type=int, default=542)
    parser.add_argument("--last", type=int, default=650)
    args = parser.parse_args()
    print(json.dumps(analyze(args.csv, args.first, args.last), indent=2))
