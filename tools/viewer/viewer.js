// KINETIC behaviour viewer: plays back the game's own recorded runs
// (tools/viewer/build_data.py output) - the bodies, the centre of mass, the
// contacts and the controller each frame, the RAGDOLL events on the
// timeline, and the verdicts the tests judge by.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const CHAINS = [
  ['pelvis', 'spine', 'spine1', 'spine2', 'neck', 'head'],
  ['spine2', 'left_clavicle', 'left_upper_arm', 'left_forearm', 'left_hand'],
  ['spine2', 'right_clavicle', 'right_upper_arm', 'right_forearm', 'right_hand'],
  ['pelvis', 'left_thigh', 'left_calf', 'left_foot'],
  ['pelvis', 'right_thigh', 'right_calf', 'right_foot'],
];
const THICK = { pelvis: 7, spine: 7, spine1: 7, spine2: 6, neck: 4, head: 4 };
const MODE_COLOR = {
  standing: 0x7fd17f, bracing: 0x9fd1ff, stumbling: 0xf2c94c, falling: 0xff6b6b, fallen: 0xc0392b,
  gettingup: 0xb28dff, getting_up: 0xb28dff, blendback: 0xb28dff, blend_back: 0xb28dff,
};
const TAG_COLOR = (tag) => {
  let h = 0;
  for (const c of tag) h = (h * 31 + c.charCodeAt(0)) >>> 0;
  return `hsl(${h % 360} 70% 62%)`;
};

// Game units, Z up -> three, Y up.
const V = (p) => new THREE.Vector3(p[0], p[2], -p[1]);

const state = {
  catalog: null, run: null, caseInfo: null, data: null, ghost: null, ghostOffset: [0, 0, 0],
  t: 0, playing: false, speed: 1, follow: true, trail: true, lastPelvis: null,
};

// ---------------------------------------------------------------- scene ---
const stage = $('stage');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
stage.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x121920);
scene.fog = new THREE.Fog(0x121920, 1500, 5000);
const camera = new THREE.PerspectiveCamera(45, 1, 1, 20000);
camera.position.set(250, 180, 250);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdfefff, 0x303840, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.2);
sun.position.set(400, 900, 300);
scene.add(sun);
const grid = new THREE.GridHelper(8000, 160, 0x34424f, 0x222c36);
scene.add(grid);
const fixtures = new THREE.Group();
scene.add(fixtures);

function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
  drawTimeline();
}
addEventListener('resize', resize);

class Skeleton {
  constructor(opacity, tint) {
    this.group = new THREE.Group();
    this.tint = tint;
    this.mat = new THREE.MeshStandardMaterial({ color: 0x7fd17f, roughness: .6, transparent: opacity < 1, opacity });
    this.footMat = [0, 1].map(() => new THREE.MeshStandardMaterial({ color: 0x7fd17f, transparent: opacity < 1, opacity }));
    this.cyl = new THREE.CylinderGeometry(1, 1, 1, 10);
    this.ball = new THREE.SphereGeometry(1, 16, 12);
    this.bones = [];
    this.joints = {};
    scene.add(this.group);
  }
  build(bodies) {
    this.group.clear();
    this.bones = [];
    this.joints = {};
    for (const chain of CHAINS) {
      const present = chain.filter((b) => bodies.includes(b));
      for (let i = 1; i < present.length; i++) {
        const m = new THREE.Mesh(this.cyl, this.mat);
        this.group.add(m);
        this.bones.push([present[i - 1], present[i], m]);
      }
    }
    for (const b of bodies) {
      const foot = b === 'left_foot' ? 0 : b === 'right_foot' ? 1 : -1;
      const m = new THREE.Mesh(this.ball, foot >= 0 ? this.footMat[foot] : this.mat);
      const r = b === 'head' ? 10 : b.endsWith('foot') ? 5 : b.endsWith('hand') ? 4 : b === 'pelvis' ? 7 : 3.2;
      m.scale.setScalar(r);
      this.group.add(m);
      this.joints[b] = m;
    }
  }
  pose(data, i, offset, live) {
    const at = (b) => {
      const p = data.bodies[b] && data.bodies[b][i];
      return p ? V([p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]]) : null;
    };
    for (const [b, m] of Object.entries(this.joints)) {
      const p = at(b);
      m.visible = !!p;
      if (p) m.position.copy(p);
    }
    for (const [a, b, m] of this.bones) {
      const pa = at(a), pb = at(b);
      m.visible = !!(pa && pb);
      if (!m.visible) continue;
      const d = pb.clone().sub(pa);
      const len = d.length();
      m.position.copy(pa).addScaledVector(d, .5);
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
      const r = THICK[a] && THICK[b] ? Math.min(THICK[a], THICK[b]) : 2.6;
      m.scale.set(r, len, r);
    }
    const mode = (data.mode[i] || '').toLowerCase();
    this.mat.color.setHex(this.tint ?? MODE_COLOR[mode] ?? 0xdddddd);
    const feet = data.feet[i] || [0, 0];
    this.footMat.forEach((fm, k) => fm.color.setHex(this.tint ?? (feet[k] ? 0x3ddc84 : 0x6b7785)));
    this.group.visible = live;
  }
}

const body = new Skeleton(1, null);
const ghost = new Skeleton(.4, 0xff9f43);
const com = new THREE.Mesh(new THREE.SphereGeometry(4.5, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffd23f }));
scene.add(com);
const comDrop = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0xffd23f, dashSize: 4, gapSize: 4 }));
scene.add(comDrop);
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffd23f, transparent: true, opacity: .55 }));
scene.add(trail);
const contacts = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xff4d6d, size: 6, sizeAttenuation: false }));
scene.add(contacts);

// -------------------------------------------------------------- loading ---
async function getJSON(url) {
  const r = await fetch(url, { cache: 'no-cache' });
  if (!r.ok) throw new Error(url + ': ' + r.status);
  return r.json();
}

async function boot() {
  try {
    state.catalog = await getJSON('data/catalog.json');
  } catch (e) {
    $('empty').textContent = 'No viewer data yet: run tools/viewer/build_data.py --latest 5';
    return;
  }
  try {
    const floor = await getJSON('data/floor.json');
    const pos = [];
    for (const t of floor) {
      pos.push(...V(t.slice(0, 3)).toArray(), ...V(t.slice(3, 6)).toArray(), ...V(t.slice(6, 9)).toArray());
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.computeVertexNormals();
    fixtures.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0x8a96a3, roughness: .9, flatShading: true })));
    fixtures.add(new THREE.LineSegments(new THREE.EdgesGeometry(g, 20), new THREE.LineBasicMaterial({ color: 0x55626f })));
  } catch (e) { /* no fixtures */ }
  const runSel = $('run');
  for (const r of state.catalog.runs) {
    const o = document.createElement('option');
    o.value = r.id;
    o.textContent = `${r.label} (${r.cases.length})`;
    runSel.append(o);
  }
  const hash = new URLSearchParams(location.hash.slice(1));
  runSel.value = hash.get('run') || state.catalog.runs[0]?.id;
  runSel.onchange = () => selectRun(runSel.value);
  await selectRun(runSel.value, hash.get('case'));
}

async function selectRun(id, caseName) {
  state.run = state.catalog.runs.find((r) => r.id === id) || state.catalog.runs[0];
  listCases();
  const first = state.run.cases.find((c) => c.name === caseName) || state.run.cases[0];
  if (first) await selectCase(first);
}

function listCases() {
  const box = $('cases');
  const q = $('search').value.toLowerCase();
  box.innerHTML = '';
  const fams = {};
  for (const c of state.run.cases) {
    if (q && !c.name.toLowerCase().includes(q)) continue;
    (fams[c.family] ||= []).push(c);
  }
  for (const [fam, list] of Object.entries(fams)) {
    const h = document.createElement('div');
    h.className = 'fam';
    h.textContent = fam;
    box.append(h);
    for (const c of list) {
      const d = document.createElement('div');
      d.className = 'case' + (state.caseInfo?.name === c.name ? ' sel' : '');
      const ok = c.status === 'passed', fell = c.fell === true;
      const color = fell ? 'var(--bad)' : ok ? 'var(--ok)' : c.status ? 'var(--warn)' : 'var(--dim)';
      d.innerHTML = `<span class="dot" style="background:${color}"></span><span class="nm"></span>` +
        (c.film ? '<span class="tag">film</span>' : '');
      d.querySelector('.nm').textContent = c.name.replace(/^PLAY-01 /, '');
      d.title = `${c.name}\n${c.status || ''}${fell ? ' - fell' : ''}`;
      d.onclick = () => selectCase(c);
      box.append(d);
    }
  }
}
$('search').oninput = listCases;

async function selectCase(c) {
  state.caseInfo = c;
  state.data = await getJSON('data/cases/' + c.file);
  $('empty').hidden = true;
  $('hud').hidden = false;
  body.build(Object.keys(state.data.bodies));
  state.t = 0;
  state.lastPelvis = null;
  location.hash = new URLSearchParams({ run: state.run.id, case: c.name }).toString();
  listCases();
  fillGhostOptions();
  await selectGhost($('ghost').value);
  fillRight();
  frameCamera();
  drawTimeline();
  update(true);
}

function fillGhostOptions() {
  const sel = $('ghost');
  const keep = sel.value;
  sel.innerHTML = '<option value="">-</option>';
  for (const r of state.catalog.runs) {
    if (r.id === state.run.id) continue;
    const c = r.cases.find((x) => x.name === state.caseInfo.name);
    if (!c) continue;
    const o = document.createElement('option');
    o.value = r.id + '|' + c.file;
    o.textContent = r.label;
    sel.append(o);
  }
  sel.value = [...sel.options].some((o) => o.value === keep) ? keep : '';
}

async function selectGhost(v) {
  state.ghost = null;
  if (v) {
    const file = v.split('|')[1];
    state.ghost = await getJSON('data/cases/' + file);
    ghost.build(Object.keys(state.ghost.bodies));
    // Aligned at the start: the ghost's first pelvis on this case's.
    const a = state.data.bodies.pelvis?.[0], b = state.ghost.bodies.pelvis?.[0];
    state.ghostOffset = a && b ? [a[0] - b[0], a[1] - b[1], 0] : [0, 0, 0];
  }
  update(true);
}
$('ghost').onchange = (e) => selectGhost(e.target.value);

// ---------------------------------------------------------- right panel ---
function fmt(v) {
  if (v === null || v === undefined) return '-';
  if (typeof v === 'number') return Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2).replace(/\.?0+$/, '');
  if (typeof v === 'boolean') return v ? 'yes' : 'no';
  return String(v);
}

const LABELS = {
  fell: 'fell', steps: 'steps taken', skid: 'feet skid (u)', slide: 'body slide (u)', steady: 'steady after (s)',
  tilt: 'max torso tilt (deg)', over: 'went over the edge', rise: 'height change (u)', td: 'touchdown at (s)',
  inv: 'went upside down', rolled: 'chest down then up', headv: 'head speed near ground', jolt: 'landing jolt (g)',
  travel: 'travel after touchdown (u)', roll: 'roll result', contacts: 'stair first contacts', edges: 'near a riser edge',
  bothOff: 'both feet off (s)', float: 'height over stairs p90', flat: 'height on the flat p90',
  swings: 'stair swings', late: 'late retargets', lifts: 'lifts', overrun: 'overrun lifts', reach: 'landing reaches',
};

function table(obj) {
  const rows = Object.entries(obj || {}).filter(([k]) => !['frames', 'trace'].includes(k));
  if (!rows.length) return '';
  return '<table>' + rows.map(([k, v]) => `<tr><td>${LABELS[k] || k}</td><td>${fmt(v)}</td></tr>`).join('') + '</table>';
}

function fillRight() {
  const d = state.data, v = d.verdict || {};
  $('title').textContent = d.name;
  const b = [];
  if (v.status) b.push(`<span class="badge ${v.status === 'passed' ? 'ok' : 'bad'}">${v.status}</span>`);
  if (v.behaviour?.fell) b.push('<span class="badge bad">fell</span>');
  b.push(`<span class="badge">${d.family}</span>`, `<span class="badge">${d.duration.toFixed(1)} s</span>`,
    `<span class="badge">${Object.keys(d.bodies).length} bodies</span>`, `<span class="badge">${d.run}</span>`);
  $('badges').innerHTML = b.join('');
  let html = '';
  if (v.checks) {
    html += '<table>' + v.checks.map(([n, val, ok]) =>
      `<tr><td>${n}</td><td class="${ok ? '' : 'bad'}">${val} ${ok ? '' : '(FAIL)'}</td></tr>`).join('') + '</table>';
  }
  if (v.behaviour) html += '<h3>Behaviour metrics</h3>' + table(v.behaviour);
  if (v.stairs) html += '<h3>Stairs metrics</h3>' + table(v.stairs);
  $('verdict').innerHTML = html || '<span style="color:var(--dim)">No verdict recorded for this case.</span>';
  const film = $('film');
  $('film-box').hidden = !d.film;
  $('film-note').textContent = d.film_note && d.film_note !== 'this run' ? 'Footage from ' + d.film_note + ', not this recording.' : '';
  if (d.film) film.src = 'files/' + d.film; else film.removeAttribute('src');
  const ev = $('events');
  ev.innerHTML = '';
  d.events.forEach((e, k) => {
    const row = document.createElement('div');
    row.dataset.k = k;
    row.innerHTML = `<span style="color:${TAG_COLOR(e[1])}">${e[0].toFixed(2)} ${e[1]}</span>`;
    row.title = e[2];
    row.onclick = () => seek(e[0]);
    ev.append(row);
  });
}

// ------------------------------------------------------------- playback ---
function frameAt(data, t) {
  const ts = data.t;
  let lo = 0, hi = ts.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (ts[mid] <= t) lo = mid; else hi = mid - 1;
  }
  return lo;
}

function frameCamera() {
  const p = state.data.bodies.pelvis?.[0] || state.data.com[0];
  const target = V(p);
  const yaw = state.data.yaw[0] || 0;
  // Behind-and-beside the body's start facing, looking at the pelvis.
  const side = new THREE.Vector3(Math.cos(yaw), 0, Math.sin(yaw));
  camera.position.copy(target).addScaledVector(side, 320).add(new THREE.Vector3(0, 120, 0));
  controls.target.copy(target);
  controls.update();
}

function sideView() {
  if (!state.data) return;
  const i = frameAt(state.data, state.t);
  const p = state.data.bodies.pelvis?.[i] || state.data.com[i];
  const target = V(p);
  const yaw = state.data.yaw[i] || 0;
  const right = new THREE.Vector3(Math.cos(yaw), 0, Math.sin(yaw));
  camera.position.copy(target).addScaledVector(right, 300).add(new THREE.Vector3(0, 40, 0));
  controls.target.copy(target);
  controls.update();
}

function update(force) {
  const d = state.data;
  if (!d) return;
  const i = frameAt(d, state.t);
  body.pose(d, i, [0, 0, 0], true);
  const c = V(d.com[i]);
  com.position.copy(c);
  // The centre of mass dropped to the lowest foot (the support's level).
  const feet = ['left_foot', 'right_foot'].map((b) => d.bodies[b]?.[i]).filter(Boolean);
  const low = feet.length ? Math.min(...feet.map((f) => f[2])) - 6 : d.com[i][2] - 90;
  comDrop.geometry.setFromPoints([c, V([d.com[i][0], d.com[i][1], low])]);
  comDrop.computeLineDistances();
  if (state.trail) {
    const from = Math.max(0, i - 180);
    trail.geometry.setFromPoints(d.com.slice(from, i + 1).map(V));
  }
  trail.visible = state.trail;
  const cts = d.contacts[i] || [];
  contacts.geometry.setAttribute('position', new THREE.Float32BufferAttribute(cts.flatMap((p) => V(p).toArray()), 3));
  if (state.ghost) {
    const g = frameAt(state.ghost, state.t);
    ghost.pose(state.ghost, g, state.ghostOffset, true);
  } else ghost.group.visible = false;
  // Follow: move the camera with the pelvis.
  const pelvis = d.bodies.pelvis?.[i] ? V(d.bodies.pelvis[i]) : c;
  if (state.follow && state.lastPelvis) {
    const delta = pelvis.clone().sub(state.lastPelvis);
    camera.position.add(delta);
    controls.target.add(delta);
  }
  state.lastPelvis = pelvis;
  // Readouts.
  const seg = [...d.segments].reverse().find((s) => s[0] <= state.t + 1e-6);
  const up = d.up[i];
  const v = d.speed[i] || [0, 0, 0];
  const f = d.feet[i] || [0, 0];
  $('hud').innerHTML = [
    `t        ${state.t.toFixed(2)} s`,
    `segment  ${seg ? seg[1] : '-'}`,
    `mode     ${d.mode[i] || '-'}`,
    `tilt     ${(Math.acos(Math.max(-1, Math.min(1, up))) * 180 / Math.PI).toFixed(0)} deg`,
    `speed    ${Math.hypot(v[0], v[1]).toFixed(0)} u/s  vz ${v[2].toFixed(0)}`,
    `feet     ${f[0] ? 'L' : '-'} ${f[1] ? 'R' : '-'}  contacts ${(d.contacts[i] || []).length}`,
  ].join('<br>').replace(/ /g, '&nbsp;');
  $('time').textContent = `${state.t.toFixed(2)} / ${d.duration.toFixed(2)} s`;
  const evs = d.events;
  for (const row of $('events').children) {
    const e = evs[+row.dataset.k];
    row.classList.toggle('now', Math.abs(e[0] - state.t) < 0.12);
  }
  syncFilm(force);
  drawTimeline();
}

function syncFilm(force) {
  const film = $('film');
  if (!state.data?.film || !film.src) return;
  film.playbackRate = Math.max(.0625, state.speed);
  if (state.playing) {
    if (film.paused) film.play().catch(() => {});
    if (Math.abs(film.currentTime - state.t) > .2) film.currentTime = state.t;
  } else {
    if (!film.paused) film.pause();
    if (force || Math.abs(film.currentTime - state.t) > .05) film.currentTime = Math.min(state.t, film.duration || state.t);
  }
}

function seek(t) {
  if (!state.data) return;
  state.t = Math.max(0, Math.min(t, state.data.duration));
  update(true);
}

let last = performance.now();
function tick(now) {
  const dt = Math.min(.1, (now - last) / 1000);
  last = now;
  if (state.playing && state.data) {
    state.t += dt * state.speed;
    if (state.t >= state.data.duration) {
      state.t = state.data.duration;
      setPlaying(false);
    }
    update(false);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

function setPlaying(on) {
  state.playing = on;
  $('play').textContent = on ? 'Pause' : 'Play';
  $('play').classList.toggle('on', on);
  if (on && state.data && state.t >= state.data.duration - 1e-3) state.t = 0;
  syncFilm(true);
}

$('play').onclick = () => setPlaying(!state.playing);
$('back').onclick = () => { setPlaying(false); seek(state.t - 1 / 60); };
$('fwd').onclick = () => { setPlaying(false); seek(state.t + 1 / 60); };
$('speed').onchange = (e) => { state.speed = +e.target.value; syncFilm(true); };
$('follow').onclick = (e) => { state.follow = !state.follow; e.target.classList.toggle('on', state.follow); };
$('trail').onclick = (e) => { state.trail = !state.trail; e.target.classList.toggle('on', state.trail); update(true); };
$('side-view').onclick = sideView;
addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'Space') { e.preventDefault(); setPlaying(!state.playing); }
  else if (e.code === 'ArrowLeft') { setPlaying(false); seek(state.t - (e.shiftKey ? .5 : 1 / 60)); }
  else if (e.code === 'ArrowRight') { setPlaying(false); seek(state.t + (e.shiftKey ? .5 : 1 / 60)); }
  else if (e.key === 'f') $('follow').click();
  else if (e.key === 's') sideView();
  else if (e.key === 't') $('trail').click();
});

// ------------------------------------------------------------- timeline ---
const tl = $('timeline');
function drawTimeline() {
  const d = state.data;
  const w = tl.clientWidth, h = tl.clientHeight;
  tl.width = w * devicePixelRatio;
  tl.height = h * devicePixelRatio;
  const g = tl.getContext('2d');
  g.scale(devicePixelRatio, devicePixelRatio);
  g.fillStyle = '#10161d';
  g.fillRect(0, 0, w, h);
  if (!d) return;
  const x = (t) => (t / Math.max(d.duration, 1e-3)) * (w - 2) + 1;
  // Segments.
  d.segments.forEach((s, k) => {
    const x0 = x(s[0]), x1 = x(d.segments[k + 1]?.[0] ?? d.duration);
    g.fillStyle = k % 2 ? '#1a232d' : '#1f2a36';
    g.fillRect(x0, 0, x1 - x0, 22);
    g.fillStyle = '#9fb0c0';
    g.font = '11px system-ui';
    g.fillText(s[1] || '', x0 + 4, 15, Math.max(0, x1 - x0 - 6));
  });
  // Controller mode strip.
  for (let i = 0; i < d.t.length; i++) {
    const c = MODE_COLOR[(d.mode[i] || '').toLowerCase()] ?? 0x555555;
    g.fillStyle = '#' + c.toString(16).padStart(6, '0');
    g.fillRect(x(d.t[i]), 24, Math.max(1, x(d.t[i + 1] ?? d.duration) - x(d.t[i]) + .5), 8);
  }
  // Events.
  const rows = {};
  let next = 0;
  for (const e of d.events) {
    if (!(e[1] in rows)) rows[e[1]] = next++ % 4;
    g.fillStyle = TAG_COLOR(e[1]);
    g.fillRect(x(e[0]) - 1, 36 + rows[e[1]] * 8, 3, 7);
  }
  // Playhead.
  g.fillStyle = '#ffffff';
  g.fillRect(x(state.t) - 1, 0, 2, h);
}
let dragging = false;
const seekFromMouse = (e) => {
  if (!state.data) return;
  const r = tl.getBoundingClientRect();
  seek(((e.clientX - r.left) / r.width) * state.data.duration);
};
tl.onmousedown = (e) => { dragging = true; setPlaying(false); seekFromMouse(e); };
addEventListener('mousemove', (e) => {
  if (dragging) seekFromMouse(e);
  else if (e.target === tl && state.data) {
    const r = tl.getBoundingClientRect();
    const t = ((e.clientX - r.left) / r.width) * state.data.duration;
    const near = state.data.events.filter((ev) => Math.abs(ev[0] - t) < state.data.duration / 150);
    tl.title = near.map((ev) => `${ev[0].toFixed(2)} ${ev[2]}`).join('\n');
  }
});
addEventListener('mouseup', () => { dragging = false; });

resize();
requestAnimationFrame(tick);
boot();
