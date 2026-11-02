import numpy as np
lines=open('source-footfall/files/data/ragdoll/reference-loops.txt').read().split('\n')
i=0;clips={}
while i<len(lines):
  l=lines[i]
  if l.startswith('clip '):
    w=l.split(); n=int(w[2]); a=np.array([[float(x) for x in lines[i+1+k].split()] for k in range(n)]).reshape(n,18,3)
    clips[w[1]]=a; i+=n+1
  else: i+=1
def curves(a):
  n=len(a); A,T=a[:,6],a[:,7]
  pitch=np.degrees(np.arctan2(A[:,2]-T[:,2], T[:,1]-A[:,1]))
  low=min(np.percentile(T[:,2],5),np.percentile(A[:,2],5))
  contact=np.minimum(T[:,2],A[:,2]-(np.percentile(A[:,2],5)-np.percentile(T[:,2],5)))<low+0.025
  flat=np.median(pitch[contact & (A[:,2]<np.percentile(A[:,2],5)+0.02)])
  rel=pitch-flat
  # lift-off = contact -> no contact
  off=[j for j in range(n) if contact[j-1] and not contact[j]][0]
  on=[j for j in range(n) if not contact[j-1] and contact[j]][0]
  sw=(on-off)%n; st=n-sw
  t=np.arange(3*n); r3=np.tile(rel,3)
  swing=np.interp(off+n+np.linspace(0,sw,11), t, r3)
  stance=np.interp(on+n+np.linspace(0,st,11), t, r3)
  return swing, stance, sw/n
for k in ('walk_fwd','run_fwd','armedwalk_fwd','armedrun_fwd'):
  s,st,d=curves(clips[k])
  print('%-14s swing share %.2f' % (k,d))
  print('   swing ', ' '.join('%4.0f'%v for v in s))
  print('   stance', ' '.join('%4.0f'%v for v in st))
