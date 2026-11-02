import csv,json,shlex,math,hashlib,argparse
from pathlib import Path
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import lsq_linear, minimize_scalar
parser=argparse.ArgumentParser(description='A63 command/actual calf and foot clearance audit; fixed humanoid geometry, not swept collision acceptance.')
parser.add_argument('evidence_directory',type=Path)
E=parser.parse_args().evidence_directory
profile_path=Path(__file__).resolve().parents[1]/'files/data/ragdoll/humanoid-v1.yaml'
profile=yaml.safe_load(profile_path.read_text())
by_name={b['name']:b for b in profile['bodies']}
for side in ('left','right'):
 assert by_name[side+'_calf']['shape']['radius']==3.5
 assert by_name[side+'_calf']['shape']['fit_to_bone']==side+'_foot'
 assert by_name[side+'_foot']['shape']['half_extents']==[4.,8.,2.5]
profile_hash=hashlib.sha256(profile_path.read_bytes()).hexdigest()

def analyze(fps):
 d=E/f'fps-a63-immediate-native-turn-candidate-{fps}-replay'
 context=E/f'fps-a63-immediate-native-turn-candidate-{fps}-context.txt'
 lines=context.read_text().splitlines();scale=float(lines[1].split()[1]);assert scale==1
 verification=json.loads((d/'witness-verification.json').read_text());assert verification['exact_recorded_physical_match']
 assert verification['driver_result']['warmup_frames']==1
 bones=[shlex.split(x) for x in lines[7:27]]
 assert [b[0] for b in bones[14:20]]==['left_thigh','left_calf','left_foot','right_thigh','right_calf','right_foot']
 bp=[np.array(list(map(float,b[2:5]))) for b in bones]
 bq=[R.from_quat(list(map(float,b[5:9]))) for b in bones]
 w={(int(x['frame']),int(x['body'])):x for x in csv.DictReader((d/'world-bodies.csv').open())}
 c={(int(x['step']),int(x['body'])):x for x in csv.DictReader((d/'startup-commands.csv').open())}
 rows={int(x['step']):x for x in csv.DictReader((d/'offline.csv').open())}
 pose=lambda f,b:(np.array([float(w[f,b][a]) for a in 'xyz']),R.from_quat([float(w[f,b]['q'+a]) for a in 'xyzw']))
 def capsule(b,p,q):
  span=bp[b+1]-bp[b];length=np.linalg.norm(span);radius=3.5
  direction=q.apply(bq[b].inv().apply(span/length))
  return p+direction*radius,p+direction*(length-radius)
 def clearance(a,b):
  a0,a1=a;b0,b1=b
  v=np.column_stack((a1-a0,-(b1-b0)))
  sol=lsq_linear(v,b0-a0,bounds=(0,1),tol=1e-12)
  return float(np.linalg.norm(a0-b0+v@sol.x)-7)
 def capsule_box(cap,boxp,boxq):
  a,b=[boxq.inv().apply(x-boxp) for x in cap]
  half=np.array([4.,8.,2.5])
  def distance(t):return np.linalg.norm(np.maximum(np.abs(a+t*(b-a))-half,0))
  sol=minimize_scalar(distance,bounds=(0,1),method='bounded',options={'xatol':1e-12})
  return float(min(distance(0),distance(1),sol.fun)-3.5)
 out=[]
 for step in range(582,597):
  if rows[step]['reference_stage']!='swing' or rows[step]['reference_stance_hip_body']!='17':continue
  f=step-2 # one warmup: pre-solve physical capture; same phase as contact-generation manifold
  hip,hq=pose(f,14);knee,kq=pose(f,15);stance,sq=pose(f,18)
  target=R.from_quat([float(c[step,14]['child_world_q'+a]) for a in 'xyzw'])
  # Builder hinge axis is plane cross twist, with twist along parent->child.
  twist=bp[15]-bp[14];twist/=np.linalg.norm(twist)
  ref=np.array([0.,0.,1.]) if abs(twist[2])<.9 else np.array([1.,0.,0.])
  plane=np.cross(twist,ref);plane/=np.linalg.norm(plane)
  axis=np.cross(plane,twist);localaxis=bq[14].inv().apply(axis)
  bind=bq[14].inv()*bq[15]
  diff=kq*bind.inv()*hq.inv()
  dq=diff.as_quat();measured=2*math.atan(np.dot(dq[:3],hq.apply(localaxis))/dq[3])
  theta=float(c[step,15]['hinge'])
  residual=abs(abs(theta-measured)-float(c[step,15]['angular_error']))
  if residual>1e-4:raise ValueError(('hinge reconstruction',fps,step,residual,theta,measured))
  targetknee=hip+target.apply(bq[14].inv().apply(bp[15]-bp[14]))
  targetq=target*R.from_rotvec(localaxis*theta)*bind
  foot,fq=pose(f,16)
  targetfoot=targetknee+targetq.apply(bq[15].inv().apply(bp[16]-bp[15]))
  relativefoot=kq.inv()*R.from_quat([float(c[step,16]['child_world_q'+a]) for a in 'xyzw'])
  targetfootq=targetq*relativefoot
  actualfoot= capsule_box(capsule(18,stance,sq),foot,fq)
  goalfoot= capsule_box(capsule(18,stance,sq),targetfoot,targetfootq)
  actual=clearance(capsule(15,knee,kq),capsule(18,stance,sq))
  goal=clearance(capsule(15,targetknee,targetq),capsule(18,stance,sq))
  # Measured joint anchor residual separates numerical joint stretch from target geometry.
  anchor=np.linalg.norm(knee-(hip+hq.apply(bq[14].inv().apply(bp[15]-bp[14]))))
  out.append(dict(step=step,actual_calf_clearance=actual,commanded_calf_clearance=goal,actual_foot_stance_calf_clearance=actualfoot,commanded_foot_stance_calf_clearance=goalfoot,knee_angle_residual=residual,measured_knee_anchor_gap=float(anchor)))
 return dict(fps=fps,profile_sha256=profile_hash,scope='Pre-solve fitted calf capsules and swing foot box; ideal commanded hip/knee/ankle against measured stance calf. Hardcoded verified profile radii3.5 and foot half-extents4,8,2.5. Does not test foot-foot, thigh, or swept-path clearance.',sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (context,d/'world-bodies.csv',d/'startup-commands.csv')},samples=out)
result=[analyze(f) for f in (60,120)]
(E/'a87-clearance.json').write_text(json.dumps(result,indent=2))
for run in result:
 print(run['fps'],[(r['step'],round(r['actual_calf_clearance'],3),round(r['commanded_calf_clearance'],3),round(r['actual_foot_stance_calf_clearance'],3),round(r['commanded_foot_stance_calf_clearance'],3)) for r in run['samples']])
