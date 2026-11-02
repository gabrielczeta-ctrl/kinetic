# Torso rocking vs the animation: chest (spine2) and pelvis pitch/roll tilt range (p5-p95, deg) and rate (median deg/s).
import numpy as np, gzip, json, glob, sys
from scipy.spatial.transform import Rotation as R
M=lambda r,n,k: np.column_stack([r['state']['bodies'][n][k][a] for a in 'xyz'])
segs=sys.argv[1].split(',')
for run in sys.argv[2:]:
  line=[]
  for seg in segs:
    vals={}
    for q in sorted(glob.glob('runtime-LOOK-%s/shard-*/*.player-diagnostics.jsonl.gz'%run)):
      rows=[json.loads(l) for l in gzip.open(q,'rt')]
      fr=[r for r in rows if r.get('kind')=='frame' and 'bodies' in r.get('state',{})]
      tests=sorted(set(r['test'] for r in fr))
      for test in tests:
        rr=[r for r in fr if r['test']==test and r['segment']==seg][20:]
        if len(rr)<40: continue
        t=np.array([r['simulation_time'] for r in rr]); dt=np.median(np.diff(t))
        y=rr[0]['gameplay_yaw']; f=np.array([np.sin(y),np.cos(y),0]); l=np.array([-np.cos(y),np.sin(y),0])
        for body in ('spine2','pelvis'):
          for k in ('rotation','targetRotation'):
            Rs=R.from_matrix([M(r,body,k) for r in rr]); m=Rs.mean()
            rel=np.degrees((Rs*m.inv()).as_rotvec())
            w=np.degrees((Rs[4:]*Rs[:-4].inv()).as_rotvec()/(4*dt))
            vals.setdefault((body,k),[]).append((np.ptp(np.percentile(rel@l,[5,95])),np.median(np.abs(w@l)),np.ptp(np.percentile(rel@f,[5,95]))))
    if not vals: continue
    c=np.mean(vals[('spine2','rotation')],0); ca=np.mean(vals[('spine2','targetRotation')],0); p=np.mean(vals[('pelvis','rotation')],0)
    line.append('%s chest pitch %4.1f (anim %3.1f) rate %3.0f (anim %3.0f) roll %4.1f (anim %4.1f) pelvis pitch %4.1f' % (seg.replace('c ',''),c[0],ca[0],c[1],ca[1],c[2],ca[2],p[0]))
  print('%-7s' % run, ' | '.join(line))
