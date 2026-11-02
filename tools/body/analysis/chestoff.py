# Chest (spine2) pitch minus animation target, mean±std per loadout for walk/run.
import numpy as np, gzip, json, glob, sys
from scipy.spatial.transform import Rotation as R
M=lambda r,n,k: np.column_stack([r['state']['bodies'][n][k][a] for a in 'xyz'])
for run in sys.argv[1:]:
  out=[]
  for q in sorted(glob.glob('runtime-LOOK-%s/shard-*/*.player-diagnostics.jsonl.gz'%run)):
    rows=[json.loads(l) for l in gzip.open(q,'rt')]
    fr=[r for r in rows if r.get('kind')=='frame' and 'bodies' in r.get('state',{})]
    for test in sorted(set(r['test'] for r in fr)):
      t=[r for r in fr if r['test']==test]; y=t[0]['gameplay_yaw']; l=np.array([-np.cos(y),np.sin(y),0])
      s=test.split()[1][:5]
      for seg in ('c walk','c run'):
        rr=[r for r in t if r['segment']==seg][20:]
        if not rr: continue
        e=np.degrees((R.from_matrix([M(r,'spine2','rotation') for r in rr])*R.from_matrix([M(r,'spine2','targetRotation') for r in rr]).inv()).as_rotvec()@l)
        s+=' %s %+5.1f±%.1f' % (seg[2:],np.mean(e),np.std(e))
      out.append(s)
  print('%-6s' % run, ' | '.join(out))
