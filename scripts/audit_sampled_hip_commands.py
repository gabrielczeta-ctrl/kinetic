from pathlib import Path
import csv,math,json,hashlib
from scipy.spatial.transform import Rotation as R
import argparse
parser=argparse.ArgumentParser(description='Compare hip commands in world coordinates and validate pre-solve frame pairing; exploratory audit, not acceptance.')
parser.add_argument('evidence_directory',type=Path)
E=parser.parse_args().evidence_directory
reports=[]
for label,folder in [('enabled','fps-a97-native-turn-candidate-60-replay'),('control','fps-a63-immediate-native-turn-candidate-60-replay')]:
 d=E/folder
 verification=json.loads((d/'witness-verification.json').read_text())
 assert verification['exact_recorded_physical_match']
 assert verification['driver_result']['warmup_frames']==1
 rows={int(x['step']):x for x in csv.DictReader((d/'offline.csv').open())};world={(int(x['frame']),int(x['body'])):x for x in csv.DictReader((d/'world-bodies.csv').open())};commands=list(csv.DictReader((d/'startup-commands.csv').open()));last={};out=[]
 for c in commands:
  s=int(c['step']);b=int(c['body']);p=last.get(b);last[b]=c
  if b not in (14,17) or p is None or not(80<=s<=731) or not c['child_world_qw'] or not p['child_world_qw']:continue
  q=R.from_quat([float(c['child_world_q'+k]) for k in 'xyzw']);old=R.from_quat([float(p['child_world_q'+k]) for k in 'xyzw']);actual=R.from_quat([float(world[s-2,b]['q'+k]) for k in 'xyzw'])
  residual=abs((q*actual.inv()).magnitude()-float(c['angular_error']))
  if residual>1e-3:raise ValueError((label,s,b,residual))
  out.append({'step':s,'body':b,'jump':(q*old.inv()).magnitude(),'error':float(c['angular_error']),'reconstruction_residual':residual,'stage':rows[s]['reference_stage'],'prior_stage':rows[s-1]['reference_stage'],'stance':rows[s]['reference_stance_hip_body'],'prior_stance':rows[s-1]['reference_stance_hip_body'],'up_before':float(rows[s-1]['pelvis_up']),'up_after':float(rows[s]['pelvis_up']),'orientation_mode_changed':c['world_orientation']!=p['world_orientation']})
 report={'label':label,'source':str(d),'sha256':{name:hashlib.sha256((d/name).read_bytes()).hexdigest() for name in ('offline.csv','world-bodies.csv','startup-commands.csv','witness-verification.json')},'max_error_reconstruction_residual':max(x['reconstruction_residual'] for x in out),'samples':out};reports.append(report)
 print(label,'max reconstruction residual',report['max_error_reconstruction_residual'])
 print(json.dumps(sorted([x for x in out if 545<=x['step']<=690],key=lambda x:x['jump'],reverse=True)[:8],indent=1))
(E/'a98-sampled-commands.json').write_text(json.dumps(reports,indent=2))
