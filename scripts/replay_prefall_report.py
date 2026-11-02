"""Summarize an already matched physical replay; does not establish parity.

All step labels are scenario CSV steps. Plan fields are pre-solve commands;
body/support fields are post-solve observations, not simultaneous inputs.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def report(path, fall_step):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    selected = [r for r in rows if int(r["step"]) <= fall_step]
    if not selected or int(selected[-1]["step"]) != fall_step:
        raise ValueError("fall step is missing")
    if any(int(b["step"]) != int(a["step"]) + 1 for a, b in zip(selected, selected[1:])):
        raise ValueError("noncontiguous scenario steps")
    # This axis-specific report must not silently label turns as lateral drift.
    active = [r for r in selected if float(r["request_y"]) > 0]
    if not active or any(abs(float(r["request_x"])) > 1e-6 for r in active):
        raise ValueError("requires forward +Y request with zero X")
    last_nonpositive = max(int(r["step"]) for r in selected if float(r["com_velocity_x"]) <= 0)
    drift = [r for r in selected if int(r["step"]) > last_nonpositive]
    transfers = []
    start = None
    for index, row in enumerate(selected):
        if row["reference_stage"] == "double_support":
            if start is None:
                start = index
        elif start is not None:
            if start == 0:
                raise ValueError("missing observation before first transfer command")
            first, before, last = selected[start], selected[start - 1], selected[index - 1]
            integral = sum(float(selected[j]["reference_requested_acceleration_x"])
                * (float(selected[j]["time"]) - float(selected[j - 1]["time"]))
                for j in range(start, index))
            transfers.append({
                "first_command_step": int(first["step"]), "last_command_step": int(last["step"]),
                "duration_seconds": float(last["time"]) - float(before["time"]),
                "before_vx": float(before["com_velocity_x"]),
                "after_vx": float(last["com_velocity_x"]),
                "observed_delta_vx": float(last["com_velocity_x"]) - float(before["com_velocity_x"]),
                "integrated_requested_ax": integral,
            })
            start = None
    return {
        "source": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "scope": "descriptive +Y forward replay; no causal or parity assertion",
        "alignment": "pre-solve plan fields; post-solve physical observations",
        "fall_step_supplied": fall_step,
        "positive_lateral_velocity_tail": {
            "start_step": int(drift[0]["step"]), "end_step": fall_step,
            "elapsed_seconds": float(drift[-1]["time"]) - float(drift[0]["time"]),
            "lateral_displacement": float(drift[-1]["com_x"]) - float(drift[0]["com_x"]),
            "end_vx": float(drift[-1]["com_velocity_x"]),
        },
        "double_support_intervals": transfers,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--fall-step", type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(report(args.csv, args.fall_step), indent=2))
