from pathlib import Path
import csv,json,math,yaml
import numpy as np
import argparse,hashlib
parser=argparse.ArgumentParser(description='A101 phase-aligned stance momentum bookkeeping; exploratory, not acceptance.')
parser.add_argument('evidence_directory',type=Path)
E=parser.parse_args().evidence_directory
profile_path=Path(__file__).resolve().parents[1]/'files/data/ragdoll/humanoid-v1.yaml'
profile=yaml.safe_load(profile_path.read_text());masses=np.array([75*b['mass_fraction'] for b in profile['bodies']]);damping=np.array([b.get('material',{}).get('linear_damping',.05) for b in profile['bodies']]);dt=float(np.float32(1/60));g=float(np.float32(np.float32(8.96)*np.float32(69.99125109)))
reports=[]
for label,folder,start in [('enabled','fps-a97-native-turn-candidate-60-replay',601),('control','fps-a63-immediate-native-turn-candidate-60-replay',596)]:
 d=E/folder
 proof=json.loads((d/'witness-verification.json').read_text())
 assert proof['exact_recorded_physical_match'] and proof['driver_result']['warmup_frames']==1
 context=E/(folder.removesuffix('-replay')+'-context.txt')
 assert float(context.read_text().splitlines()[1].split()[0])==75
 r={int(x['step']):x for x in csv.DictReader((d/'offline.csv').open())};w={(int(x['frame']),int(x['body'])):x for x in csv.DictReader((d/'world-bodies.csv').open())};out=[]
 for s in range(start,start+17):
  x=r[s];before=np.array([[float(w[s-2,b]['v'+k]) for k in 'xyz'] for b in range(20)]);after=np.array([[float(w[s-1,b]['v'+k]) for k in 'xyz'] for b in range(20)])
  free=(before+[0,0,-g*dt])*(1-damping[:,None]*dt);expected=np.sum(masses[:,None]*(after-free),axis=0);recorded=np.array([float(x['solver_linear_impulse_'+k]) for k in 'xyz']);residual=np.linalg.norm(expected-recorded)
  actual_acc=(np.sum(masses[:,None]*(after-before),axis=0)/75/dt);request=np.array([float(x['reference_requested_acceleration_'+k]) for k in 'xy']);normal=float(x['left_contact_normal_impulse'])+float(x['right_contact_normal_impulse'])
  assert residual<1e-3,(label,s,residual)
  assert x['solver_velocity_limit']=='0' and x['free_velocity_limit']=='0'
  assert x['upright_applied']=='0' and x['swing_control_applied']=='0'
  # force_* stores an inactive proposal when upright_applied is false.
  weighted_cop=[sum(float(x[side+'_cop_'+k])*float(x[side+'_contact_normal_impulse']) for side in ('left','right') if float(x[side+'_contact_normal_impulse'])>0)/normal for k in 'xy']
  com=np.array([float(x['solver_reference_com_'+k]) for k in 'xyz']);moment=np.array([float(x['solver_angular_impulse_'+k]) for k in 'xyz'])
  wrench_cop=[com[0]+(-moment[1]-com[2]*recorded[0])/recorded[2],com[1]+(moment[0]-com[2]*recorded[1])/recorded[2]]
  # Both captures use a flat ground plane at z=0; joint-anchor moment is
  # already removed by the solver impulse telemetry. This is a bookkeeping
  # check; the solver impulse channel itself is derived from momentum.
  assert abs(normal-recorded[2])<.1
  assert np.linalg.norm(np.array(wrench_cop)-weighted_cop)<.1
  out.append({'step':s,'phase_step':s-start,'impulse_reconstruction_error':float(residual),'ground_normal_sum':normal,'solver_vertical_impulse':recorded[2],'normal_closure_error':abs(normal-recorded[2]),'actual_acceleration':actual_acc.tolist(),'requested_acceleration':request.tolist(),'up':float(x['pelvis_up']),'ground_cop':weighted_cop,'wrench_cop':wrench_cop,'left_cop':[float(x['left_cop_'+k]) for k in 'xy'],'com':[float(x['solver_reference_com_'+k]) for k in 'xy'],'angular_momentum':[float(x['centroidal_angular_momentum_'+k]) for k in 'xyz'],'angular_impulse':[float(x['solver_angular_impulse_'+k]) for k in 'xyz']})
 reports.append({'label':label,'scope':'momentum/ground-wrench bookkeeping, not independent dynamics validation or matched-state causality','sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (profile_path,context,d/'offline.csv',d/'world-bodies.csv')},'samples':out});print(label,'max closure',max(x['impulse_reconstruction_error'] for x in out),'normalclosure',max(x['normal_closure_error'] for x in out))
 for x in out[::3]:print(x['step'],'req',np.round(x['requested_acceleration'],1),'actual',np.round(x['actual_acceleration'],1),'up',round(x['up'],4),'L',np.round(x['angular_momentum'],0),'cop-com',np.round(np.array(x['ground_cop'])-x['com'],1))
(E/'a101-stance-response.json').write_text(json.dumps(reports,indent=2))
