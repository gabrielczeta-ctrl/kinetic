import numpy as np, gzip, json, glob, sys
P=lambda b,n,k: np.array(b[n][k])
for run in sys.argv[1:]:
  out=[]
  for p in sorted(glob.glob('runtime-LOOK-%s/shard-*/*.player-diagnostics.jsonl.gz'%run)):
    rows=[json.loads(l) for l in gzip.open(p,'rt')]
    for lo in ('fists','dagger','longsword','claymore','warhammer'):
      rr=[r for r in rows if r.get('kind')=='frame' and (' '+lo+' ') in r.get('test','') and r.get('segment')=='c run' and 'bodies' in r.get('state',{})][60:]
      if not rr: continue
      y=rr[0]['gameplay_yaw']; f=np.array([np.sin(y),np.cos(y),0]); L=72.0
      row=[]
      for k in ('position','targetPosition'):
        H=np.array([P(r['state']['bodies'],'left_thigh',k) for r in rr]); K=np.array([P(r['state']['bodies'],'left_calf',k) for r in rr]); A=np.array([P(r['state']['bodies'],'left_foot',k) for r in rr])
        row+= [np.percentile((K-H)@f/L,98),np.percentile((A-H)@f/L,98),-np.percentile((A-H)@f/L,2),np.percentile((A-H)[:,2]/L,98)+1]
      out.append(row); print("   %-9s" % lo, " ".join("%.2f" % v for v in row))
  a=np.array(out).mean(0)
  print('%-7s phys kneeFwd %.2f ankleFwd %.2f back %.2f heelUp %.2f | target %.2f %.2f %.2f %.2f' % (run,*a))
