# Upper body tracking vs the animation: position error (median/p90 units) of head, hands, forearms,
# and head pitch range/rate vs anim, per segment, averaged over loadouts.
import numpy as np, gzip, json, glob, sys
from scipy.spatial.transform import Rotation as R
M=lambda r,n,k: np.column_stack([r['state']['bodies'][n][k][a] for a in 'xyz'])
segs=sys.argv[1].split(',')
for run in sys.argv[2:]:
  acc={}
  for q in sorted(glob.glob('runtime-LOOK-%s/shard-*/*.player-diagnostics.jsonl.gz'%run)):
    rows=[json.loads(l) for l in gzip.open(q,'rt')]
    fr=[r for r in rows if r.get('kind')=='frame' and 'bodies' in r.get('state',{})]
    for test in sorted(set(r['test'] for r in fr)):
      t=[r for r in fr if r['test']==test]; y=t[0]['gameplay_yaw']; l=np.array([-np.cos(y),np.sin(y),0])
      for seg in segs:
        rr=[r for r in t if r['segment']==seg][20:]
        if len(rr)<30 or 'head' not in rr[0]['state']['bodies']: continue
        dt=np.median(np.diff([r['simulation_time'] for r in rr]))
        for n in ('head','left_hand','right_hand'):
          d=np.array([np.linalg.norm(np.array(r['state']['bodies'][n]['position'])-np.array(r['state']['bodies'][n]['targetPosition'])) for r in rr])
          acc.setdefault((seg,n),[]).append(d)
        for k in ('rotation','targetRotation'):
          Rs=R.from_matrix([M(r,'head',k) for r in rr]); m=Rs.mean()
          rel=np.degrees((Rs*m.inv()).as_rotvec()@l)
          w=np.degrees((Rs[4:]*Rs[:-4].inv()).as_rotvec()/(4*dt))@l
          acc.setdefault((seg,'hp',k),[]).append((np.ptp(np.percentile(rel,[5,95])),np.median(np.abs(w))))
  out=[]
  for seg in segs:
    if (seg,'head') not in acc: continue
    s=seg.replace('c ','')+':'
    for n in ('head','left_hand','right_hand'):
      d=np.concatenate(acc[(seg,n)]); s+=' %s %.1f/%.1f' % (n.replace('_hand','H'),np.median(d),np.percentile(d,90))
    hp=np.mean(acc[(seg,'hp','rotation')],0); ha=np.mean(acc[(seg,'hp','targetRotation')],0)
    s+=' headpitch %.1f (anim %.1f) rate %.0f (anim %.0f)' % (hp[0],ha[0],hp[1],ha[1])
    out.append(s)
  print('%-6s' % run, ' | '.join(out))
