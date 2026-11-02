"""Compare pre-solve transfer requests with post-solve velocity changes.

Descriptive accounting only: requested/projected acceleration is not a promise
that joint motors deliver it. Each command is paired with its preceding solved
observation and the resulting observation, never with a forward difference.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def analyze(path, first=60, last=160):
    path = Path(path)
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if any(int(b["step"]) != int(a["step"]) + 1 for a, b in zip(rows, rows[1:])):
        raise ValueError("Noncontiguous steps")
    intervals = []
    current = []
    for before, row in zip(rows, rows[1:]):
        selected = first <= int(row["step"]) <= last and row["reference_stage"] == "double_support"
        if selected:
            dt = float(row["time"]) - float(before["time"])
            if not math.isfinite(dt) or dt <= 0:
                raise ValueError("Invalid timestep")
            current.append((before, row, dt))
        if current and (not selected or int(row["step"]) == last):
            intervals.append(current)
            current = []
    if current:
        intervals.append(current)
    report = []
    for group in intervals:
        entry = {"first_command_step": int(group[0][1]["step"]),
                 "last_command_step": int(group[-1][1]["step"]),
                 "touches_window_boundary": int(group[0][1]["step"]) == first
                 or int(group[-1][1]["step"]) == last,
                 "duration": sum(dt for _, _, dt in group),
                 "support_planned_frames": sum(row["joint_support_planned"] == "1" for _, row, _ in group),
                 "frames": len(group)}
        for axis in ("x", "y"):
            def integrate(column):
                values = [float(row[column]) for _, row, _ in group]
                if not all(map(math.isfinite, values)):
                    raise ValueError("Nonfinite acceleration")
                return sum(v * sample[2] for v, sample in zip(values, group))
            observed = float(group[-1][1]["com_velocity_" + axis]) - float(group[0][0]["com_velocity_" + axis])
            if not math.isfinite(observed):
                raise ValueError("Nonfinite observed velocity change")
            entry[axis] = {"requested_delta_v": integrate("reference_requested_acceleration_" + axis),
                           "projected_delta_v": integrate("joint_support_acceleration_" + axis)
                           if entry["support_planned_frames"] == len(group) else None,
                           "observed_delta_v": observed}
        report.append(entry)
    return {"source": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "window": [first, last], "scope": "Per-transfer acceleration accounting, not causal proof or gait acceptance",
            "transfers": report}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--first", type=int, default=60)
    parser.add_argument("--last", type=int, default=160)
    args = parser.parse_args()
    print(json.dumps(analyze(args.csv, args.first, args.last), indent=2))
