#!/usr/bin/env python3
"""Build a self-contained HTML viewer for the RXN-01 offline controller.

Runs the disabled RagdollCoreReactionViewerTest with the viewer environment
variable set, reads the JSON trace it writes, and produces one HTML file with
two animated scenes:

  * Walker    - the real reference controller on the synthetic body (side and
                top views, COM, foot support, gait stage).
  * Reactions - the real reaction pipeline on the same body with forward
                kinematics, behavior labels, pose-correction authority,
                stabilization strength, speed scale and limb flags.

No network access, no external assets: the data is embedded in the page.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>RXN-01 Poise - offline viewer</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font-family: ui-monospace, Consolas, monospace; background: #10141a; color: #d8dee9; }
  header { padding: 10px 16px; border-bottom: 1px solid #263041; display: flex; gap: 16px; align-items: baseline; }
  h1 { font-size: 15px; margin: 0; color: #8fbcbb; }
  .sub { color: #7b8794; font-size: 12px; }
  .tabs { padding: 8px 16px; display: flex; gap: 8px; }
  .tabs button { background: #1b2330; color: #d8dee9; border: 1px solid #2e3a4d; padding: 6px 14px; cursor: pointer; }
  .tabs button.active { background: #2e6b74; border-color: #4a9aa6; }
  main { display: flex; gap: 16px; padding: 0 16px 16px; flex-wrap: wrap; }
  canvas { background: #0b0f14; border: 1px solid #263041; }
  .panel { min-width: 280px; max-width: 340px; }
  .row { margin: 6px 0; font-size: 12px; }
  .bar { height: 10px; background: #1b2330; border: 1px solid #2e3a4d; position: relative; }
  .bar > div { height: 100%; background: #4a9aa6; }
  .bar.red > div { background: #bf616a; }
  .bar.green > div { background: #a3be8c; }
  .bar.yellow > div { background: #ebcb8b; }
  .controls { padding: 8px 16px; display: flex; gap: 12px; align-items: center; font-size: 12px; }
  input[type=range] { width: 360px; }
  .flag { display: inline-block; margin-right: 6px; padding: 1px 6px; border: 1px solid #2e3a4d; color: #7b8794; }
  .flag.on { color: #10141a; background: #ebcb8b; border-color: #ebcb8b; }
  .legend { font-size: 11px; color: #7b8794; padding: 0 16px 8px; }
  .event { color: #bf616a; }
</style>
</head>
<body>
<header>
  <h1>RXN-01 Poise - offline controller viewer</h1>
  <span class="sub" id="meta"></span>
</header>
<div class="tabs">
  <button id="tabWalker" class="active">Walker (reference controller)</button>
  <button id="tabReactions">Reactions (pipeline)</button>
</div>
<div class="controls">
  <button id="play">pause</button>
  <input id="slider" type="range" min="0" max="1000" value="0">
  <span id="clock">0.00 s</span>
  <span id="speedLabel">1x</span>
  <button id="speed">speed</button>
</div>
<main>
  <canvas id="side" width="720" height="460"></canvas>
  <canvas id="top" width="360" height="460"></canvas>
  <div class="panel" id="panel"></div>
</main>
<div class="legend">bones: spine chain grey, arms blue, legs teal, feet green when supported, COM yellow. Reactions: highlighted limb = active behavior.</div>
<script>
const DATA = __DATA__;

const bones = DATA.bodyNames;
const parents = DATA.parents;
const walker = DATA.walker;
const reactions = DATA.reactions;

const state = { scene: 'walker', index: 0, playing: true, speed: 1, last: 0 };

const side = document.getElementById('side').getContext('2d');
const top = document.getElementById('top').getContext('2d');
const panel = document.getElementById('panel');

function accent(name) {
  if (/head|neck/.test(name)) return '#b48ead';
  if (/arm|hand|clavicle/.test(name)) return '#81a1c1';
  if (/thigh|calf|foot/.test(name)) return '#88c0d0';
  return '#9aa5b1';
}

function behaviorAccent(name, frame) {
  if (frame.cover && /head|neck|arm|hand/.test(name)) return '#bf616a';
  if ((frame.cradleL && /left_(upper_arm|forearm|hand)/.test(name))
      || (frame.cradleR && /right_(upper_arm|forearm|hand)/.test(name))) return '#ebcb8b';
  if ((frame.limpL && /left_(thigh|calf|foot)/.test(name))
      || (frame.limpR && /right_(thigh|calf|foot)/.test(name))) return '#a3be8c';
  return accent(name);
}

function bounds(frames) {
  let minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9, minX = 1e9, maxX = -1e9;
  for (const frame of frames) for (const p of frame.bodies) {
    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    minZ = Math.min(minZ, p[2]); maxZ = Math.max(maxZ, p[2]);
    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
  }
  return { minX, maxX, minY, maxY, minZ, maxZ };
}
const sceneBounds = { walker: bounds(walker.frames), reactions: bounds(reactions.frames) };

function drawScene(ctx, canvas, points, frame, labels, sceneB) {
  const b = sceneB;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const sideView = ctx === side;
  const span = sideView ? Math.max(b.maxY - b.minY, b.maxZ - b.minZ, 1)
                        : Math.max(b.maxX - b.minX, b.maxY - b.minY, 1);
  const scale = (sideView ? Math.min(canvas.width, canvas.height - 20)
                          : Math.min(canvas.width, canvas.height - 20)) / span * 0.85;
  const originY = sideView ? (b.minY + b.maxY) / 2 : (b.minX + b.maxX) / 2;
  const originZ = sideView ? (b.minZ + b.maxZ) / 2 : (b.minY + b.maxY) / 2;
  const project = (p) => sideView
    ? [canvas.width / 2 + (p[1] - originY) * scale, canvas.height / 2 - (p[2] - originZ) * scale]
    : [canvas.width / 2 + (p[0] - originY) * scale, canvas.height / 2 + (p[1] - originZ) * scale];

  // ground line
  ctx.strokeStyle = '#263041';
  ctx.beginPath();
  const ground = project(sideView ? [0, b.minY, 0] : [0, 0, b.minZ]);
  ctx.moveTo(0, project(sideView ? [0, b.minY, 0] : [b.minX, 0, 0])[1]);
  ctx.lineTo(canvas.width, ground[1]);
  ctx.stroke();

  for (let i = 0; i < points.length; ++i) {
    const parent = parents[i];
    const color = labels ? behaviorAccent(bones[i], frame) : accent(bones[i]);
    if (parent >= 0) {
      const a = project(points[parent]);
      const c = project(points[i]);
      ctx.strokeStyle = color;
      ctx.lineWidth = /spine|head|neck/.test(bones[i]) ? 5 : 3;
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(c[0], c[1]); ctx.stroke();
    }
    const p = project(points[i]);
    ctx.fillStyle = color;
    ctx.fillRect(p[0] - 2, p[1] - 2, 4, 4);
  }
  if (frame.com) {
    const p = project(frame.com);
    ctx.fillStyle = '#ebcb8b';
    ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 6.283); ctx.fill();
  }
}

function bar(label, value, max, cls) {
  const pct = Math.max(0, Math.min(1, value / max)) * 100;
  return `<div class="row">${label}: ${value.toFixed(3)}<div class="bar ${cls}"><div style="width:${pct}%"></div></div></div>`;
}

function render() {
  const slider = document.getElementById('slider');
  if (state.scene === 'walker') {
    const frames = walker.frames;
    state.index = Math.min(frames.length - 1, Math.max(0, state.index));
    const frame = frames[state.index];
    drawScene(side, document.getElementById('side'), frame.bodies, frame, false, sceneBounds.walker);
    drawScene(top, document.getElementById('top'), frame.bodies, frame, false, sceneBounds.walker);
    const stageNames = ['inactive','settling','awaiting','transfer','swing','touchdown','unloading'];
    panel.innerHTML = `<div class="row"><b>reference walker</b></div>
      <div class="row">t: ${frame.t.toFixed(2)} s  step ${state.index}</div>
      <div class="row">stage: ${stageNames[frame.stage] || frame.stage}</div>
      <div class="row">requested speed: ${frame.speed.toFixed(1)} u/s</div>
      <div class="row">support: ${frame.left ? 'left ' : ''}${frame.right ? 'right' : ''}${!frame.left && !frame.right ? 'airborne' : ''}</div>`;
    document.getElementById('clock').textContent = frame.t.toFixed(2) + ' s';
    slider.value = String(Math.round(1000 * state.index / (frames.length - 1)));
    document.getElementById('meta').textContent = `${frames.length} frames, 60 Hz, ${bones.length} bodies`;
  } else {
    const frames = reactions.frames;
    state.index = Math.min(frames.length - 1, Math.max(0, state.index));
    const frame = frames[state.index];
    drawScene(side, document.getElementById('side'), frame.bodies, frame, true, sceneBounds.reactions);
    drawScene(top, document.getElementById('top'), frame.bodies, frame, true, sceneBounds.reactions);
    const phases = ['idle','impact','stagger','catch','fall','get-up','resuming'];
    panel.innerHTML = `<div class="row"><b>reaction pipeline</b></div>
      <div class="row">t: ${frame.t.toFixed(2)} s</div>
      <div class="row">behavior: ${frame.behavior} (intensity ${frame.intensity.toFixed(2)})</div>
      <div class="row">phase: ${phases[frame.phase] || frame.phase}</div>
      ${bar('pose-correction authority', frame.authority, 1, 'yellow')}
      ${bar('stabilization strength', frame.stability, 1, 'green')}
      ${bar('speed scale', frame.speed, 1, 'blue')}
      <div class="row"><span class="flag ${frame.cover ? 'on' : ''}">cover face</span>
        <span class="flag ${frame.cradleL ? 'on' : ''}">cradle L</span>
        <span class="flag ${frame.cradleR ? 'on' : ''}">cradle R</span>
        <span class="flag ${frame.limpL ? 'on' : ''}">limp L</span>
        <span class="flag ${frame.limpR ? 'on' : ''}">limp R</span></div>
      <div class="row event">events: head @1.00 s, left forearm @2.33 s, left calf @3.00 s</div>`;
    document.getElementById('clock').textContent = frame.t.toFixed(2) + ' s';
    slider.value = String(Math.round(1000 * state.index / (frames.length - 1)));
    document.getElementById('meta').textContent = `${frames.length} frames, 60 Hz, real pipeline + FK`;
  }
}

function tick(time) {
  if (state.playing) {
    const dt = Math.min(0.05, (time - state.last) / 1000);
    const frames = state.scene === 'walker' ? walker.frames.length : reactions.frames.length;
    const seconds = 1 / 60;
    state.index += (dt * state.speed) / seconds;
    if (state.index >= frames - 1) state.index = 0;
  }
  state.last = time;
  render();
  requestAnimationFrame(tick);
}

document.getElementById('play').onclick = (event) => {
  state.playing = !state.playing;
  event.target.textContent = state.playing ? 'pause' : 'play';
};
document.getElementById('speed').onclick = () => {
  state.speed = state.speed === 1 ? 0.25 : state.speed === 0.25 ? 0.5 : 1;
  document.getElementById('speedLabel').textContent = state.speed + 'x';
};
document.getElementById('slider').oninput = (event) => {
  const frames = state.scene === 'walker' ? walker.frames.length : reactions.frames.length;
  state.index = (Number(event.target.value) / 1000) * (frames - 1);
  state.playing = false;
  document.getElementById('play').textContent = 'play';
};
function setScene(scene) {
  state.scene = scene;
  state.index = 0;
  document.getElementById('tabWalker').className = scene === 'walker' ? 'active' : '';
  document.getElementById('tabReactions').className = scene === 'reactions' ? 'active' : '';
}
document.getElementById('tabWalker').onclick = () => setScene('walker');
document.getElementById('tabReactions').onclick = () => setScene('reactions');
window.onerror = (message, source, line) => {
  const el = document.getElementById('panel');
  if (el) el.innerHTML = '<div class="row" style="color:#bf616a">viewer error: ' + message + ' @' + line + '</div>' + el.innerHTML;
};
render();
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def write_png(path: Path, width: int, height: int, pixels: bytearray) -> None:
    """Minimal RGB PNG writer (no external libraries)."""
    import struct
    import zlib

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw += pixels[y * stride:(y + 1) * stride]
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    path.write_bytes(png)


class Sheet:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self.pixels = bytearray([11, 15, 20] * width * height)

    def put(self, x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            index = (y * self.width + x) * 3
            self.pixels[index:index + 3] = bytes(color)

    def line(self, x0: float, y0: float, x1: float, y1: float, color: tuple[int, int, int],
             thickness: int = 2) -> None:
        steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for step in range(steps + 1):
            t = step / steps if steps else 0.0
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            for dx in range(-thickness, thickness + 1):
                for dy in range(-thickness, thickness + 1):
                    if dx * dx + dy * dy <= thickness * thickness:
                        self.put(int(x) + dx, int(y) + dy, color)

    def dot(self, x: float, y: float, color: tuple[int, int, int], radius: int = 3) -> None:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    self.put(int(x) + dx, int(y) + dy, color)

    def frame(self, x0: int, y0: int, w: int, h: int, color: tuple[int, int, int] = (38, 48, 65)) -> None:
        for x in range(x0, x0 + w):
            self.put(x, y0, color)
            self.put(x, y0 + h - 1, color)
        for y in range(y0, y0 + h):
            self.put(x0, y, color)
            self.put(x0 + w - 1, y, color)


def accent(name: str) -> tuple[int, int, int]:
    if "head" in name or "neck" in name:
        return (180, 142, 173)
    if "arm" in name or "hand" in name or "clavicle" in name:
        return (129, 161, 193)
    if "thigh" in name or "calf" in name or "foot" in name:
        return (136, 192, 208)
    return (154, 165, 177)


def behavior_accent(name: str, frame: dict) -> tuple[int, int, int]:
    if frame.get("cover") and ("head" in name or "neck" in name or "arm" in name):
        return (191, 97, 106)
    if (frame.get("cradleL") and name.startswith("left_") and "arm" in name) or \
       (frame.get("cradleR") and name.startswith("right_") and "arm" in name):
        return (235, 203, 139)
    if (frame.get("limpL") and name.startswith("left_") and ("thigh" in name or "calf" in name)) or \
       (frame.get("limpR") and name.startswith("right_") and ("thigh" in name or "calf" in name)):
        return (163, 190, 140)
    return accent(name)


def scene_bounds(frames: list[dict]) -> dict:
    ys = [p[1] for frame in frames for p in frame["bodies"]]
    zs = [p[2] for frame in frames for p in frame["bodies"]]
    return {"minY": min(ys), "maxY": max(ys), "minZ": min(zs), "maxZ": max(zs)}


def render_cell(sheet: Sheet, x0: int, y0: int, w: int, h: int, frame: dict,
                labels: bool, scene: dict) -> None:
    sheet.frame(x0, y0, w, h)
    points = frame["bodies"]
    parents = DATA_PARENTS
    names = DATA_NAMES
    min_y, max_y = scene["minY"], scene["maxY"]
    min_z, max_z = scene["minZ"], scene["maxZ"]
    span = max(max_y - min_y, max_z - min_z, 1.0)
    scale = min(w - 24, h - 24) / span
    origin_y = (min_y + max_y) / 2
    origin_z = (min_z + max_z) / 2

    def project(point):
        return (x0 + w / 2 + (point[1] - origin_y) * scale,
                y0 + h / 2 - (point[2] - origin_z) * scale)

    ground = project([0, min_y, 0])
    sheet.line(x0 + 2, ground[1], x0 + w - 3, ground[1], (30, 38, 51), 1)
    for index, point in enumerate(points):
        parent = parents[index]
        color = behavior_accent(names[index], frame) if labels else accent(names[index])
        px, py = project(point)
        if parent >= 0:
            ax, ay = project(points[parent])
            thickness = 3 if ("spine" in names[index] or "head" in names[index]) else 2
            sheet.line(ax, ay, px, py, color, thickness)
        sheet.dot(px, py, color, 2)
    if frame.get("com"):
        cx, cy = project(frame["com"])
        sheet.dot(cx, cy, (235, 203, 139), 4)


def build_sheet(data: dict, path: Path) -> None:
    global DATA_PARENTS, DATA_NAMES
    DATA_NAMES = data["bodyNames"]
    DATA_PARENTS = data["parents"]
    columns = 6
    cell_w, cell_h = 240, 260
    sheet = Sheet(columns * cell_w, 2 * cell_h + 8)
    walker_frames = data["walker"]["frames"]
    walker_scene = scene_bounds(walker_frames)
    picks = [walker_frames[min(len(walker_frames) - 1, i * (len(walker_frames) - 1) // (columns - 1))]
             for i in range(columns)]
    for column, frame in enumerate(picks):
        render_cell(sheet, column * cell_w, 0, cell_w, cell_h, frame, labels=False,
                    scene=walker_scene)
    reaction_frames = data["reactions"]["frames"]
    reaction_scene = scene_bounds(reaction_frames)
    event_indices = [0, 62, 72, 142, 182, 228]
    for column, index in enumerate(event_indices):
        index = min(index, len(reaction_frames) - 1)
        render_cell(sheet, column * cell_w, cell_h + 8, cell_w, cell_h,
                    reaction_frames[index], labels=True, scene=reaction_scene)
    write_png(path, sheet.width, sheet.height, sheet.pixels)



def svg_frame(points, parents, names, scene, frame, labels, width=520, height=430):
    min_y, max_y = scene["minY"], scene["maxY"]
    min_z, max_z = scene["minZ"], scene["maxZ"]
    span = max(max_y - min_y, max_z - min_z, 1.0)
    scale = min(width - 30, height - 60) / span
    origin_y = (min_y + max_y) / 2
    origin_z = (min_z + max_z) / 2

    def project(point):
        return (width / 2 + (point[1] - origin_y) * scale,
                height / 2 - (point[2] - origin_z) * scale - 10)

    parts = [f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg">']
    ground = project([0, min_y, 0])
    parts.append(f'<line x1="4" y1="{ground[1]:.1f}" x2="{width - 4}" y2="{ground[1]:.1f}" '
                 f'stroke="#1e2633" stroke-width="2"/>')
    for index, point in enumerate(points):
        parent = parents[index]
        color = behavior_accent(names[index], frame) if labels else accent(names[index])
        hexcolor = "#%02x%02x%02x" % color
        px, py = project(point)
        if parent >= 0:
            ax, ay = project(points[parent])
            thickness = 5 if ("spine" in names[index] or "head" in names[index]) else 3
            parts.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{px:.1f}" y2="{py:.1f}" '
                         f'stroke="{hexcolor}" stroke-width="{thickness}" stroke-linecap="round"/>')
        parts.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="2.6" fill="{hexcolor}"/>')
    if frame.get("com"):
        cx, cy = project(frame["com"])
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="#ebcb8b"/>')
    if labels:
        behavior = frame.get("behavior", "none")
        phase = ["idle", "impact", "stagger", "catch", "fall", "get-up", "resuming"][frame.get("phase", 0)]
        parts.append(f'<text x="12" y="22" fill="#8fbcbb" font-family="monospace" font-size="16">'
                     f'{behavior} / {phase}</text>')
        parts.append(f'<rect x="12" y="32" width="{200 * frame.get("authority", 0):.0f}" height="8" fill="#ebcb8b"/>')
        parts.append(f'<rect x="12" y="44" width="{200 * frame.get("stability", 1):.0f}" height="8" fill="#a3be8c"/>')
        parts.append(f'<text x="12" y="68" fill="#7b8794" font-family="monospace" font-size="13">'
                     f't {frame.get("t", 0):.2f}s  authority {frame.get("authority", 0):.2f}  '
                     f'stability {frame.get("stability", 1):.2f}</text>')
    else:
        parts.append(f'<text x="12" y="22" fill="#8fbcbb" font-family="monospace" font-size="16">'
                     f'reference walker  t {frame.get("t", 0):.2f}s</text>')
        parts.append(f'<text x="12" y="42" fill="#7b8794" font-family="monospace" font-size="13">'
                     f'speed {frame.get("speed", 0):.1f} u/s  '
                     f'support {"L" if frame.get("left") else ""}{"R" if frame.get("right") else ""}</text>')
    parts.append("</svg>")
    return "".join(parts)


def build_css_animation(data: dict, path: Path) -> None:
    names = data["bodyNames"]
    parents = data["parents"]
    walker_frames = data["walker"]["frames"]
    reaction_frames = data["reactions"]["frames"]
    walker_scene = scene_bounds(walker_frames)
    reaction_scene = scene_bounds(reaction_frames)
    walker_frames = walker_frames[::4]
    reaction_frames = reaction_frames[::6]
    walker_slot = 100.0 / len(walker_frames)
    reaction_slot = 100.0 / len(reaction_frames)
    # Two-slot visible windows keep at least one frame opaque at all times.
    walker_window = min(95.0, walker_slot * 2.0)
    reaction_window = min(95.0, reaction_slot * 2.0)
    walker_duration = 6.0
    reaction_duration = 4.0
    parts = ["""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>RXN-01 animated (no scripts)</title>
<style>
  body { margin:0; background:#10141a; color:#d8dee9; font-family:ui-monospace,Consolas,monospace; }
  header { padding:10px 16px; border-bottom:1px solid #263041; }
  h1 { font-size:15px; margin:0; color:#8fbcbb; }
  h2 { font-size:13px; margin:14px 16px 4px; color:#8fbcbb; }
  .sub { color:#7b8794; font-size:12px; padding:0 16px; }
  .stage { position:relative; height:430px; margin:0 16px 10px; border:1px solid #263041; overflow:hidden; }
  .stage svg { position:absolute; inset:0; width:100%%; height:100%%; opacity:0; animation: slot linear infinite; }
  @keyframes slotW { 0%% {opacity:1} %s%% {opacity:0} 100%% {opacity:0} }
  @keyframes slotR { 0%% {opacity:1} %s%% {opacity:0} 100%% {opacity:0} }
  .w svg { animation-name: slotW; animation-duration: %ss; }
  .r svg { animation-name: slotR; animation-duration: %ss; }
  .events { color:#bf616a; font-size:12px; padding:0 16px 18px; }
</style></head><body>
<header><h1>RXN-01 Poise - animated offline viewer (script-free)</h1></header>
<div class="sub">Pure CSS/SVG animation: no JavaScript, no network. Each frame is one SVG shown for one slot.</div>
<h2>Reference walker (physical controller, 6 s at 45 u/s, mid-walk impulse)</h2>
<div class="stage w">""" % (walker_window, reaction_window, walker_duration, reaction_duration)]
    for index, frame in enumerate(walker_frames):
        delay = -(len(walker_frames) - 1 - index) * walker_duration / len(walker_frames)
        parts.append(svg_frame(frame["bodies"], parents, names, walker_scene, frame, False)
            .replace("<svg ", f'<svg style="animation-delay:{delay:.3f}s" ', 1))
    parts.append('</div><h2>Reaction pipeline (head hit, forearm receipt, calf receipt, recovery)</h2>')
    parts.append('<div class="stage r">')
    for index, frame in enumerate(reaction_frames):
        delay = -(len(reaction_frames) - 1 - index) * reaction_duration / len(reaction_frames)
        parts.append(svg_frame(frame["bodies"], parents, names, reaction_scene, frame, True)
            .replace("<svg ", f'<svg style="animation-delay:{delay:.3f}s" ', 1))
    parts.append('</div>')
    parts.append('<div class="events">events: head hit @1.00 s (cover face), left forearm @2.33 s '
                 '(cradle), left calf @3.00 s (limp), recovery brace @3.67 s</div>')
    parts.append("</body></html>")
    path.write_text("\n".join(parts), encoding="utf-8", newline="\n")



GIF_PALETTE = [
    (11, 15, 20), (38, 48, 65), (154, 165, 177), (180, 142, 173),
    (129, 161, 193), (136, 192, 208), (235, 203, 139), (191, 97, 106),
    (163, 190, 140), (143, 188, 187), (123, 135, 148), (216, 222, 233),
    (30, 38, 51), (255, 255, 255), (90, 100, 115), (60, 70, 85),
]


def palette_index(color) -> int:
    best, best_distance = 0, 1 << 30
    for index, entry in enumerate(GIF_PALETTE):
        distance = ((color[0] - entry[0]) ** 2 + (color[1] - entry[1]) ** 2
                    + (color[2] - entry[2]) ** 2)
        if distance < best_distance:
            best, best_distance = index, distance
    return best


class BitWriter:
    def __init__(self) -> None:
        self.data = bytearray()
        self.accumulator = 0
        self.bits = 0

    def write(self, code: int, size: int) -> None:
        self.accumulator |= code << self.bits
        self.bits += size
        while self.bits >= 8:
            self.data.append(self.accumulator & 0xFF)
            self.accumulator >>= 8
            self.bits -= 8

    def flush(self) -> None:
        if self.bits:
            self.data.append(self.accumulator & 0xFF)
            self.accumulator = 0
            self.bits = 0


def lzw_encode(indices: bytes, minimum_code_size: int = 4) -> bytes:
    clear = 1 << minimum_code_size
    end = clear + 1
    writer = BitWriter()
    table = {}
    next_code = 0
    code_size = 0

    def reset() -> None:
        nonlocal table, next_code, code_size
        table = {bytes([value]): value for value in range(clear)}
        next_code = end + 1
        code_size = minimum_code_size + 1

    reset()
    writer.write(clear, code_size)
    prefix = b""
    for value in indices:
        current = prefix + bytes([value])
        if current in table:
            prefix = current
            continue
        writer.write(table[prefix], code_size)
        if next_code < 4096:
            table[current] = next_code
            next_code += 1
            if next_code > (1 << code_size) and code_size < 12:
                code_size += 1
        else:
            writer.write(clear, code_size)
            reset()
        prefix = bytes([value])
    if prefix:
        writer.write(table[prefix], code_size)
    writer.write(end, code_size)
    writer.flush()
    return bytes(writer.data)


def write_gif(path: Path, width: int, height: int, frames: list[bytes], delay_cs: int) -> None:
    import struct

    out = bytearray(b"GIF89a")
    out += struct.pack("<HH", width, height)
    out += bytes([0x80 | 0x70 | (len(GIF_PALETTE).bit_length() - 2)])
    for color in GIF_PALETTE:
        out += bytes(color)
    # Reserve room by padding the global table to a power of two.
    for _ in range((1 << (len(GIF_PALETTE).bit_length() - 1)) - len(GIF_PALETTE)):
        out += b"\x00\x00\x00"
    out += b"\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00"
    for frame in frames:
        out += b"\x21\xf9\x04\x00"
        out += struct.pack("<H", delay_cs)
        out += b"\x00\x00"
        out += b"\x2c" + struct.pack("<HHHH", 0, 0, width, height) + b"\x00"
        out.append(4)
        data = lzw_encode(frame)
        for start in range(0, len(data), 255):
            block = data[start:start + 255]
            out.append(len(block))
            out += block
        out.append(0)
    out += b"\x3b"
    path.write_bytes(bytes(out))


def render_gif(path: Path, frames: list[dict], scene: dict, labels: bool,
               width: int = 320, height: int = 300, delay_cs: int = 10) -> None:
    encoded = []
    cache: dict[tuple[int, int, int], int] = {}
    for frame in frames:
        sheet = Sheet(width, height)
        render_cell(sheet, 0, 0, width, height, frame, labels, scene)
        indices = bytearray(width * height)
        for index in range(width * height):
            color = (sheet.pixels[index * 3], sheet.pixels[index * 3 + 1], sheet.pixels[index * 3 + 2])
            mapped = cache.get(color)
            if mapped is None:
                mapped = palette_index(color)
                cache[color] = mapped
            indices[index] = mapped
        encoded.append(bytes(indices))
    write_gif(path, width, height, encoded, delay_cs)



def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    trace = out / "rxn01-viewer.json"
    env = dict(os.environ)
    env["OPENMW_RAGDOLL_REACTION_VIEWER"] = str(trace)
    run = subprocess.run(
        [str(args.exe.resolve()), "--gtest_filter=RagdollCoreReactionViewerTest.*",
         "--gtest_also_run_disabled_tests", "--gtest_brief=1"],
        capture_output=True, text=True, env=env)
    (out / "viewer-run.log").write_text(run.stdout + run.stderr, encoding="utf-8", newline="\n")
    if not trace.is_file():
        print("viewer trace was not produced; see viewer-run.log", file=sys.stderr)
        return 1
    data = json.loads(trace.read_text(encoding="utf-8"))
    sheet_path = out / "rxn01-sheet.png"
    build_sheet(data, sheet_path)
    import base64
    sheet_b64 = base64.b64encode(sheet_path.read_bytes()).decode("ascii")
    html = TEMPLATE.replace("__DATA__", json.dumps(data, separators=(",", ":")))
    html = html.replace("</main>",
        f'</main><div style="padding:0 16px 24px"><div class="sub">Static preview (works without scripts): '
        f'<a href="{sheet_path.name}">{sheet_path.name}</a></div>'
        f'<img src="data:image/png;base64,{sheet_b64}" style="max-width:100%;border:1px solid #263041" '
        f'alt="RXN-01 static preview"></div>')
    target = out / "rxn01-viewer.html"
    target.write_text(html, encoding="utf-8", newline="\n")
    print(f"viewer written: {target} ({target.stat().st_size // 1024} KiB)")
    animated = out / "rxn01-animated.html"
    build_css_animation(data, animated)
    print(f"script-free animated page: {animated} ({animated.stat().st_size // 1024} KiB)")
    walker_gif = out / "rxn01-walker.gif"
    reaction_gif = out / "rxn01-reactions.gif"
    walker_all = data["walker"]["frames"]
    reaction_all = data["reactions"]["frames"]
    render_gif(walker_gif, walker_all[::3], scene_bounds(walker_all), False)
    render_gif(reaction_gif, reaction_all[::6], scene_bounds(reaction_all), True)
    print(f"animated gifs: {walker_gif.name} ({walker_gif.stat().st_size // 1024} KiB), "
          f"{reaction_gif.name} ({reaction_gif.stat().st_size // 1024} KiB)")
    duel_trace = out / "rxn01-duel.json"
    env = dict(os.environ)
    env["OPENMW_RAGDOLL_DUEL_VIEWER"] = str(duel_trace)
    run = subprocess.run(
        [str(args.exe.resolve()), "--gtest_filter=RagdollCoreDuelScenarioTest.DISABLED_writesDuelViewerTrace",
         "--gtest_also_run_disabled_tests", "--gtest_brief=1"],
        capture_output=True, text=True, env=env)
    (out / "duel-viewer-run.log").write_text(run.stdout + run.stderr, encoding="utf-8", newline="\n")
    if duel_trace.is_file():
        import rxn01_duelview
        duel_gif = out / "rxn01-duel.gif"
        rxn01_duelview.build_duel_gif(duel_trace, duel_gif)
        print(f"duel gif: {duel_gif.name} ({duel_gif.stat().st_size // 1024} KiB)")
        try:
            rxn01_duelview.append_duel_section(animated, duel_trace)
            print("duel section appended to the script-free animated page")
        except Exception as error:
            print("duel section skipped:", error)
    if args.open:
        subprocess.run(["cmd.exe", "/c", "start", "", str(target)], check=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
