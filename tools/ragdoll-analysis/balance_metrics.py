#!/usr/bin/env python3
"""Summarize BAL-01 RAGDOLL_BALANCE telemetry from OpenMW log files.

Reads the lines emitted under OPENMW_RAGDOLL_BALANCE_TRACE and reports, per
actor: capture-point margin (how far outside the foot segment the body is
already heading), foot target error (how far the physical feet are from the
drive pose), knee limit clamps, foot loads and the reaction offset. Phase 0
uses this to measure how much of the balance the kinematic harness is doing
before any authority is handed back to the body.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics as stats
from pathlib import Path

LINE = re.compile(r"RAGDOLL_BALANCE actor=(\S+)((?: [-a-zA-Z_]+=[-0-9.eE+]+)*)")


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(rows: list[dict[str, float]]) -> dict[str, object]:
    def column(name: str) -> list[float]:
        return [row[name] for row in rows if name in row]

    def describe(name: str) -> dict[str, float] | None:
        values = column(name)
        if not values:
            return None
        return {
            "n": len(values),
            "mean": round(stats.fmean(values), 4),
            "p10": round(percentile(values, 10), 4),
            "p50": round(percentile(values, 50), 4),
            "p90": round(percentile(values, 90), 4),
            "max": round(max(values), 4),
        }

    margins = column("margin")
    result: dict[str, object] = {
        "samples": len(rows),
        "marginUnknown": sum(1 for value in margins if value < 0),
        # A margin above half a unit is a body already outside its support.
        "outsideSupport": sum(1 for value in margins if value > 0.5),
        "rootKinematic": sum(1 for row in rows if row.get("root") == 1.0),
    }
    for field in ("comh", "comd", "margin", "lload", "rload", "lerr", "rerr", "lknee", "rknee",
            "lkneeT", "rkneeT", "react", "auth"):
        result[field] = describe(field)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+", help="openmw.log files to parse")
    parser.add_argument("--out", type=Path, help="write the summary JSON here")
    args = parser.parse_args()

    per_actor: dict[str, list[dict[str, float]]] = {}
    total = 0
    for path in args.logs:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = LINE.search(line)
            if not match:
                continue
            row: dict[str, float] = {}
            for token in match.group(2).split():
                key, _, value = token.partition("=")
                try:
                    row[key] = float(value)
                except ValueError:
                    continue
            per_actor.setdefault(match.group(1), []).append(row)
            total += 1

    summary = {
        "logs": [str(path) for path in args.logs],
        "lines": total,
        "actors": {actor: summarize(rows) for actor, rows in per_actor.items()},
    }
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    condensed = {
        actor: {
            key: value
            for key, value in actor_summary.items()
            if key in ("samples", "outsideSupport", "margin", "rerr", "rknee")
        }
        for actor, actor_summary in summary["actors"].items()
    }
    print(json.dumps(condensed, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
