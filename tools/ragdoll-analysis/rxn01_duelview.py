#!/usr/bin/env python3
"""Duel rendering for the RXN-01 offline viewer.

Consumes the trace written by
RagdollCoreDuelScenarioTest.DISABLED_writesDuelViewerTrace and produces an
animated GIF plus, optionally, a script-free CSS/SVG section for the HTML
viewer. Uses the sheet/GIF helpers from rxn01_viewer.
"""

from __future__ import annotations

import json
from pathlib import Path

from rxn01_viewer import Sheet, accent, palette_index, write_gif


def _scene_bounds(frames: list[dict]) -> dict:
    ys = [p[1] for frame in frames for side in ("left", "right") for p in frame[side]["bodies"]]
    zs = [p[2] for frame in frames for side in ("left", "right") for p in frame[side]["bodies"]]
    return {"minY": min(ys), "maxY": max(ys), "minZ": min(zs), "maxZ": max(zs)}


def _sheet_indices(sheet: Sheet, cache: dict) -> bytes:
    indices = bytearray(sheet.width * sheet.height)
    for index in range(sheet.width * sheet.height):
        color = (sheet.pixels[index * 3], sheet.pixels[index * 3 + 1], sheet.pixels[index * 3 + 2])
        mapped = cache.get(color)
        if mapped is None:
            mapped = palette_index(color)
            cache[color] = mapped
        indices[index] = mapped
    return bytes(indices)


def _draw_frame(sheet: Sheet, frame: dict, names: list[str], parents: list[int], scene: dict,
                width: int, height: int) -> None:
    min_y, max_y = scene["minY"], scene["maxY"]
    min_z, max_z = scene["minZ"], scene["maxZ"]
    span = max(max_y - min_y, max_z - min_z, 1.0)
    scale = min(width - 30, height - 90) / span
    origin_y = (min_y + max_y) / 2
    origin_z = (min_z + max_z) / 2

    def project(point):
        return (width / 2 + (point[1] - origin_y) * scale,
                height / 2 - (point[2] - origin_z) * scale + 18)

    ground = project([0, min_y, 0])
    sheet.line(4, ground[1], width - 4, ground[1], (30, 38, 51), 1)
    for side in ("left", "right"):
        actor = frame[side]
        points = actor["bodies"]
        for index, point in enumerate(points):
            parent = parents[index]
            color = accent(names[index])
            px, py = project(point)
            if parent >= 0:
                ax, ay = project(points[parent])
                thickness = 3 if ("spine" in names[index] or "head" in names[index]) else 2
                sheet.line(ax, ay, px, py, color, thickness)
            sheet.dot(px, py, color, 2)
        if actor.get("grip") and actor.get("tip"):
            gx, gy = project(actor["grip"])
            tx, ty = project(actor["tip"])
            sheet.line(gx, gy, tx, ty, (216, 222, 233), 2)
        bar_width, bar_height = 150, 10
        bar_x = 10 if side == "left" else width - 10 - bar_width
        health = max(0.0, min(1.0, actor["health"] / 40.0))
        sheet.frame(bar_x - 1, 9, bar_width + 2, bar_height + 2, (90, 100, 115))
        for x in range(int(bar_width * health)):
            for y in range(bar_height):
                sheet.put(bar_x + x, 10 + y,
                          (163, 190, 140) if actor["alive"] else (191, 97, 106))
        if actor.get("hit"):
            sheet.frame(1, 1, width - 2, height - 2, (191, 97, 106))


def build_duel_gif(trace_path: Path, out_path: Path, width: int = 520, height: int = 340,
                   delay_cs: int = 8) -> None:
    data = json.loads(trace_path.read_text(encoding="utf-8"))
    names = data["bodyNames"]
    parents = data["parents"]
    frames = data["frames"]
    scene = _scene_bounds(frames)
    step = max(1, len(frames) // 120)
    cache: dict = {}
    encoded = []
    for frame in frames[::step]:
        sheet = Sheet(width, height)
        _draw_frame(sheet, frame, names, parents, scene, width, height)
        encoded.append(_sheet_indices(sheet, cache))
    write_gif(out_path, width, height, encoded, delay_cs)


def append_duel_section(html_path: Path, trace_path: Path, duration: float = 12.0) -> None:
    data = json.loads(trace_path.read_text(encoding="utf-8"))
    names = data["bodyNames"]
    parents = data["parents"]
    frames = data["frames"][::12]
    scene = _scene_bounds(frames)
    slot = min(95.0, 200.0 / len(frames))  # two-slot windows overlap
    parts = ['<h2>Offline physical duel (real weapon bodies, solver contacts, reactions)</h2>',
             '<div class="stage d">']
    for index, frame in enumerate(frames):
        delay = -(len(frames) - 1 - index) * duration / len(frames)
        min_y, max_y = scene["minY"], scene["maxY"]
        min_z, max_z = scene["minZ"], scene["maxZ"]
        span = max(max_y - min_y, max_z - min_z, 1.0)
        scale = min(490, 360) / span
        origin_y = (min_y + max_y) / 2
        origin_z = (min_z + max_z) / 2

        def project(point):
            return (260 + (point[1] - origin_y) * scale, 210 - (point[2] - origin_z) * scale)

        svg = ['<svg viewBox="0 0 520 430" xmlns="http://www.w3.org/2000/svg">']
        for side in ("left", "right"):
            actor = frame[side]
            points = actor["bodies"]
            for body_index, point in enumerate(points):
                px, py = project(point)
                parent = parents[body_index]
                hexcolor = "#%02x%02x%02x" % accent(names[body_index])
                if parent >= 0:
                    ax, ay = project(points[parent])
                    thickness = 5 if ("spine" in names[body_index] or "head" in names[body_index]) else 3
                    svg.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{px:.1f}" y2="{py:.1f}" '
                               f'stroke="{hexcolor}" stroke-width="{thickness}" stroke-linecap="round"/>')
            if actor.get("grip") and actor.get("tip"):
                gx, gy = project(actor["grip"])
                tx, ty = project(actor["tip"])
                svg.append(f'<line x1="{gx:.1f}" y1="{gy:.1f}" x2="{tx:.1f}" y2="{ty:.1f}" '
                           f'stroke="#d8dee9" stroke-width="3"/>')
            bar_x = 12 if side == "left" else 300
            health_width = 180 * max(0.0, min(1.0, actor["health"] / 40.0))
            bar_color = "#a3be8c" if actor["alive"] else "#bf616a"
            svg.append(f'<rect x="{bar_x}" y="10" width="180" height="9" fill="#1b2330"/>')
            svg.append(f'<rect x="{bar_x}" y="10" width="{health_width:.0f}" height="9" fill="{bar_color}"/>')
            svg.append(f'<text x="{bar_x}" y="36" fill="#8fbcbb" font-family="monospace" font-size="13">'
                       f'{side} {actor["health"]:.1f} hp  {actor["behavior"]}'
                       f'{"  HIT " + actor.get("region", "") if actor.get("hit") else ""}</text>')
        svg.append("</svg>")
        rendered = "".join(svg).replace("<svg ", f'<svg style="animation-delay:{delay:.3f}s" ', 1)
        parts.append(rendered)
    parts.append("</div>")
    html = html_path.read_text(encoding="utf-8")
    css = ('<style>.stage.d svg { animation-name: slotD; animation-duration: 12s; }\n'
           f'@keyframes slotD {{ 0% {{opacity:1}} {slot}% {{opacity:0}} 100% {{opacity:0}} }}</style>\n')
    insert_at = html.find("</body>")
    html = html[:insert_at] + css + "".join(parts) + html[insert_at:]
    html_path.write_text(html, encoding="utf-8", newline="\n")
