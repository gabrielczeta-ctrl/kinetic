# Foot pitch (deg, + toes down) about the facing's left axis, relative to the standing pose.
import numpy as np, gzip, json, glob, sys
from scipy.spatial.transform import Rotation as R
seg=sys.argv[1]
M=lambda r,k: np.column_stack([r['state']['bodies']['left_foot'][k][a] for a in 'xyz'])
for run in sys.argv[2:]:
  for q in sorted(glob.glob('runtime-LOOK-%s/shard-*/*.player-diagnostics.jsonl.gz'%run)):
    rows=[json.loads(l) for l in gzip.open(q,'rt')]
    fr=[r for r in rows if r.get('kind')=='frame' and 'bodies' in r.get('state',{})]
    if not fr: continue
    lo=fr[0]['test'].split()[1]
    st=[r for r in fr if r['segment']=='c stand'][30:]
    rr=[r for r in fr if r['segment']==seg][30:]
    if not st or not rr: continue
    y=rr[0]['gameplay_yaw']; left=np.array([-np.cos(y),np.sin(y),0])
    out=[]
    for k in ('rotation','targetRotation'):
      s0=R.from_matrix([M(r,k) for r in st]).mean()
      rel=R.from_matrix([M(r,k) for r in rr])*s0.inv()
      p=np.degrees(rel.as_rotvec()@left)
      out.append(p)
    ph,tg=out
    print('%-7s %-9s phys p5 %5.0f p95 %5.0f | target p5 %5.0f p95 %5.0f | corr %.2f' % (run,lo,np.percentile(ph,5),np.percentile(ph,95),np.percentile(tg,5),np.percentile(tg,95),np.corrcoef(ph,tg)[0,1]))
