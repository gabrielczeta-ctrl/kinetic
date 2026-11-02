"""Validate a manual trace and make a self-contained bookmark/pose viewer.

The viewer redraws recorded world-space observations, not a physics replay.
Run with a manual runtime or its manual-recording directory.
"""
import argparse
import collections
import json
import math
from pathlib import Path


def inspect(directory):
    files = sorted(directory.glob('ticks-*.jsonl'), key=lambda p: int(p.stem.split('-')[-1]))
    counts, bookmarks, ticks, issues = collections.Counter(), [], set(), []
    previous = None
    dx = dy = 0
    input_mismatches = 0
    command_frames = 0
    actor_frames = 0
    notes = {}
    for path in files:
        with path.open(encoding='utf-8') as stream:
            for line_number, line in enumerate(stream, 1):
                try:
                    row = json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
                except (ValueError, json.JSONDecodeError) as error:
                    issues.append(f'{path.name}:{line_number}: {error}')
                    continue
                kind = row['kind']
                counts[kind] += 1
                if kind == 'input':
                    dx += row['dx']; dy += row['dy']
                    counts['synthetic_inputs' if row['synthetic_handler'] else 'ordinary_inputs'] += 1
                elif kind == 'bookmark':
                    bookmarks.append(row)
                elif kind == 'bookmark_note':
                    notes[row['id']] = row
                elif kind == 'tick':
                    if previous is not None and row['tick'] != previous + 1:
                        issues.append(f'Physics tick gap/reversal: {previous} -> {row["tick"]}')
                    previous = row['tick']
                    if not (math.isclose(dx, row['mouse'][0], abs_tol=1e-5)
                            and math.isclose(dy, row['mouse'][1], abs_tol=1e-5)):
                        input_mismatches += 1
                    dx = dy = 0
                    command_frames += any(b['valid'][1] for b in row['bodies'])
                    actor_frames += bool(row['actor'])
                    ticks.add(row['tick'])
    for mark in bookmarks:
        note = notes.pop(mark['id'], None)
        if note:
            if note['tick'] != mark['tick'] or not isinstance(note.get('reason'), str):
                issues.append(f'Invalid note for bookmark {mark["id"]}')
            else:
                mark['reason'] = note['reason']
        if mark['tick'] not in ticks:
            issues.append(f'Bookmark {mark["id"]} has no complete tick {mark["tick"]}')
    if not ticks: issues.append('No recorded physics ticks')
    if notes: issues.append('Notes reference missing bookmarks')
    if input_mismatches: issues.append(f'{input_mismatches} tick/input accumulation mismatches')
    if actor_frames and not command_frames:
        issues.append('Player bodies recorded but no motor commands were captured')
    summary = dict(schema=1, purpose='observation integrity only; no gameplay verdict',
                   counts=dict(counts), bookmarks=bookmarks, command_frames=command_frames,
                   issues=issues, chunks=[p.name for p in files])
    # Keep a bounded visual review around each bookmark. Full trace stays on disk.
    # Two passes avoid keeping a potentially GiB-sized session decoded in RAM.
    # The ten latest bookmarks bound viewer size; all bookmarks remain in summary.
    summary['viewer_bookmarks'] = bookmarks[-10:]
    centers = [b['tick'] for b in summary['viewer_bookmarks']] or ([max(ticks)] if ticks else [])
    selected = []
    for path in files:
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try: row = json.loads(line)
                except ValueError: continue
                if row.get('kind') == 'tick' and any(abs(row['tick'] - tick) <= 180 for tick in centers):
                    selected.append(row)
    return summary, selected


HTML = r'''<!doctype html><meta charset="utf-8"><title>PLAY-04 recorded observations</title>
<style>body{margin:24px;background:#10151d;color:#e5e9f0;font:15px system-ui}button,select,input{margin:5px;padding:7px}canvas{background:#18202d;width:100%;border-radius:8px}#pose{height:520px}#graphs{height:180px}.muted{color:#a9b7c9}label{margin-right:18px}</style>
<h1>Recorded arm observations</h1><p class="muted">Cyan: animation. Purple: motor command. Physical: green → red motor load; gray unavailable. Yellow: hand target.<br>This draws saved poses, not a simulation replay. Gaps between bookmark windows are not continuous playback.</p>
<select id="bookmark"><option value="">Bookmarks</option></select><select id="view"><option value="Side">Side (world YZ)</option><option value="Front">Front (world XZ)</option><option value="Above">Above (world XY)</option></select>
<p id="reason" style="white-space:pre-wrap"></p>
<button id="back">Previous tick</button><button id="next">Next tick</button><label><input type="checkbox" id="anim" checked>Animation</label><label><input type="checkbox" id="cmd" checked>Command</label>
<input id="tick" type="range" min="0" value="0" style="width:95%"><div id="status"></div><canvas id="pose" width="1400" height="520"></canvas><canvas id="graphs" width="1400" height="180"></canvas>
<p class="muted">Graphs: maximum hand-target error (yellow, scale 30 world units), motor utilization (green, scale 1), maximum grip gap (red, scale 3 world units). Gray vertical cursor marks the selected tick. Full values and unclipped poses remain in JSONL.</p>
<details><summary>Recording integrity</summary><pre id="summary"></pre></details>
<script>const data=PAYLOAD;const $=id=>document.getElementById(id);const fs=data.frames;const slider=$('tick');slider.max=Math.max(0,fs.length-1);$('summary').textContent=JSON.stringify(data.summary,null,2);
for(const b of data.summary.viewer_bookmarks){const o=document.createElement('option');o.value=b.id;o.textContent=`#${b.id} — tick ${b.tick}`;$('bookmark').append(o)}
$('bookmark').onchange=()=>{const b=data.summary.viewer_bookmarks.find(b=>b.id===Number($('bookmark').value));$('reason').textContent=b?`Bookmark #${b.id}: ${b.reason||'(no reason entered)'}`:'';const i=fs.findIndex(f=>f.tick===b?.tick);if(i>=0){slider.value=i;draw()}};
for(const id of ['tick','view','anim','cmd'])$(id).oninput=draw;
$('back').onclick=()=>{slider.value=Math.max(0,Number(slider.value)-1);draw()};$('next').onclick=()=>{slider.value=Math.min(fs.length-1,Number(slider.value)+1);draw()};
function rot(q,v){let [x,y,z,w]=q;let [a,b,c]=v;let tx=2*(y*c-z*b),ty=2*(z*a-x*c),tz=2*(x*b-y*a);return [a+w*tx+y*tz-z*ty,b+w*ty+z*tx-x*tz,c+w*tz+x*ty-y*tx]}
function draw(){if(!fs.length)return;let i=Number(slider.value),f=fs[i],c=$('pose').getContext('2d');c.clearRect(0,0,1400,520);let root=f.bodies.find(b=>b.name==='torso'),origin=root?.physical||[0,0,0];
function project(p){let d=p.map((v,j)=>v-origin[j]);return $('view').value==='Above'?[700+d[0]*4,260-d[1]*4]:$('view').value==='Front'?[700+d[0]*4,300-d[2]*4]:[700+d[1]*4,300-d[2]*4]}
function line(a,b,color,width=3){a=project(a);b=project(b);c.strokeStyle=color;c.lineWidth=width;c.beginPath();c.moveTo(...a);c.lineTo(...b);c.stroke()}
for(let k=0;k<3;k++){if(k===0&&!$('anim').checked||k===1&&!$('cmd').checked)continue;let key=['animation','command','physical'][k];for(let b of f.bodies){if(!b.valid[k])continue;let color=k===0?'#00daff':k===1?'#d566ff':b.util<0?'#999':`rgb(${255*Math.min(1,b.util)},${255*(1-Math.min(1,b.util))},40)`;let p=f.bodies[b.parent];if(p&&p.valid[k])line(p[key],b[key],color);if(b.name==='weapon'){let v=rot(b[key+'_q'],f.weapon_axis.map(x=>x*f.weapon_length/2));line(b[key].map((x,j)=>x-v[j]),b[key].map((x,j)=>x+v[j]),color,5)}}}
for(let j=0;j<2;j++)if(f.hand_error[j]>=0){let p=f.targets[j],q=project(p);c.fillStyle='#ffdf20';c.fillRect(q[0]-4,q[1]-4,8,8);let hand=f.bodies.find(b=>b.name===(j?'right_hand':'left_hand'));if(hand)line(hand.physical,p,'#ffdf20',1)}
$('status').textContent=`Tick ${f.tick} | actor ${f.actor} | ${f.weapon||'fists/no weapon'} | phase ${f.phase} | mouse ${f.mouse.join(', ')} | hand error ${f.hand_error.map(x=>x.toFixed(2))} | grip ${f.grip.map(x=>x.toFixed(2))}`;
let g=$('graphs').getContext('2d');g.clearRect(0,0,1400,180);let start=Math.max(0,i-180),end=Math.min(fs.length-1,i+180),n=Math.max(1,end-start);for(let a=0;a<3;a++){g.strokeStyle=['#ffdf20','#80ff50','#ff7050'][a];g.beginPath();let previous=null;for(let j=start;j<=end;j++){let r=fs[j],v=a===0?Math.max(...r.hand_error)/30:a===1?Math.max(-1,...r.bodies.map(b=>b.util)):Math.max(...r.grip)/3;let x=(j-start)/n*1400,y=170-Math.min(1,Math.max(0,v))*155;if(v<0){previous=null;continue}if(previous!==r.tick-1)g.moveTo(x,y);else g.lineTo(x,y);previous=r.tick}g.stroke()}g.strokeStyle='#bbb';g.beginPath();g.moveTo((i-start)/n*1400,0);g.lineTo((i-start)/n*1400,180);g.stroke()}
if(data.summary.viewer_bookmarks.length){$('bookmark').value=data.summary.viewer_bookmarks[0].id;$('bookmark').onchange()}else{let first=fs.findIndex(f=>f.actor);slider.value=Math.max(0,first);draw()}</script>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime', type=Path)
    args = parser.parse_args()
    directory = args.runtime / 'manual-recording' if (args.runtime / 'manual-recording').is_dir() else args.runtime
    summary, frames = inspect(directory)
    (directory / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    payload = json.dumps(dict(summary=summary, frames=frames), separators=(',', ':')).replace('<', '\\u003c')
    (directory / 'review.html').write_text(HTML.replace('PAYLOAD', payload), encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return bool(summary['issues'])


if __name__ == '__main__':
    raise SystemExit(main())
