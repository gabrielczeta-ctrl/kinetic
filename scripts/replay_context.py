"""Extract explicit rig/frame/consumed-input context; never infer an integer radix."""
import argparse,json,math,re
from pathlib import Path

def fields(line):
    return dict(re.findall(r'(\w+)=("(?:\\.|[^"\\])*"|[^\s]+)',line))

def extract(log, destination, shape_directory=None, actor_kind="player", reconstructed_poses=None, recovery_poses=None,
            raw_facing=False):
    if actor_kind not in ("player", "npc"): raise ValueError("Unknown actor kind")
    player_flag = "1" if actor_kind == "player" else "0"
    selected_actors = set()
    direct_impulses=[]
    recovery_requests=[]
    rigs=[];bones=[];frames=[];steps={};poses={};body_ids={};grounds=[];recovery_events=[]
    contact_cycles=[];contact_frames={};contact_poses={};contact_inputs={}
    for line in log.read_text(encoding='utf-8').splitlines():
        if 'RAGDOLL_REPLAY_COMBAT_IMPULSE ' in line and fields(line).get('player')==player_flag:
            raise ValueError('Combat event replay not wired; refusing to discard hit-stun event')
        if 'RAGDOLL_REPLAY_RECOVERY_REQUEST ' in line:
            f=fields(line)
            if f.get('player') == player_flag:
                if f.get('schema')!='1' or f.get('integer_radix')!='10':
                    raise ValueError('Undeclared recovery request schema or radix')
                recovery_requests.append(int(f['next_step']))
        if 'RAGDOLL_REPLAY_EXTERNAL_IMPULSE ' in line:
            f=fields(line)
            if f.get('player') == player_flag:
                if f.get('schema')!='1' or f.get('integer_radix')!='10':
                    raise ValueError('Undeclared direct impulse schema or radix')
                direct_impulses.append(f)
        if 'RAGDOLL_CONTACT_ROOT_INPUT ' in line:
            f=fields(line)
            if f.get('player')==player_flag:
                if f.get('schema')!='1' or f.get('integer_radix')!='10':
                    raise ValueError('Undeclared contact-root input schema or radix')
                step=int(f['step'])
                if step in contact_inputs: raise ValueError('Duplicate contact-root input step')
                contact_inputs[step]=f
        elif 'RAGDOLL_CONTACT_ROOT_CYCLE ' in line:
            contact_cycles.append(fields(line))
        elif 'RAGDOLL_CONTACT_ROOT_FRAME ' in line:
            f=fields(line); index=int(f['frame'])
            if index in contact_frames: raise ValueError('Duplicate contact-root frame')
            contact_frames[index]=f
        elif 'RAGDOLL_CONTACT_ROOT_POSE ' in line:
            f=fields(line); key=(int(f['frame']),int(f['body']))
            if key in contact_poses: raise ValueError('Duplicate contact-root pose')
            contact_poses[key]=f
        elif 'RAGDOLL_REPLAY_RIG ' in line: rigs.append(fields(line))
        elif 'RAGDOLL_REPLAY_BONE ' in line: bones.append(fields(line))
        elif 'RAGDOLL_REPLAY_FRAME ' in line: frames.append(fields(line))
        elif 'RAGDOLL_REPLAY_RECOVERY_FINISH ' in line:
            f=fields(line)
            if f.get('player')==player_flag:
                if f.get('integer_radix')!='10': raise ValueError('Undeclared recovery event radix')
                recovery_events.append(int(f['next_step']))
        elif 'RAGDOLL_REPLAY_GROUND ' in line: grounds.append(fields(line))
        elif 'RAGDOLL_REPLAY_INITIAL ' in line:
            f=fields(line)
            if f.get('player')==player_flag and 'body_id' in f:
                body=int(f['body'])
                if body in body_ids: raise ValueError('Duplicate initial body ID record')
                body_ids[body]=int(f['body_id'])
        elif 'RAGDOLL_REPLAY_BASE ' in line:
            f=fields(line)
            if f.get('player')==player_flag and reconstructed_poses is None:
                key=(int(f['step']),int(f['body']))
                if key in poses: raise ValueError('Duplicate consumed base pose')
                poses[key]=f
        elif 'RAGDOLL_STEP phase=applied ' in line:
            f=fields(line)
            if f.get('player')==player_flag:
                selected_actors.add(f.get('actor', '<unnamed>'))
                if len(selected_actors)>1: raise ValueError('Multiple selected actors; capture one actor in isolation')
                if f.get('integer_radix')!='10': raise ValueError('Undeclared trace radix')
                step=int(f['step'])
                if step in steps: raise ValueError('Duplicate player applied step')
                steps[step]=f
    if len(rigs)!=1 or not frames: raise ValueError('Need one complete capture context')
    rig=rigs[0];warm=int(rig['controller_steps']);first=int(frames[0]['step'])-warm
    if warm>4: raise ValueError('Long unrecorded controller history cannot be guessed')
    has_poses=bool(poses) or reconstructed_poses is not None
    version=3 if has_poses and body_ids else (2 if has_poses else 1)
    if shape_directory is not None:
        if version!=3 or len(grounds)!=1 or grounds[0]['saved']!='1':
            raise ValueError('Need exactly one saved static ground for this reconstruction')
        version=4
    if recovery_poses is not None:
        if version!=4 or reconstructed_poses is None:
            raise ValueError('Paired candidates require reconstructed poses and captured ground')
        version=5
    out=[str(version)+' '+json.dumps(rig['profile_id'])+' '+rig['profile_hash']+' '+str(warm),
         rig['mass']+' '+rig['scale']+' 0',rig['root'].replace(',',' '),rig['rotation'].replace(',',' '),
         rig['velocity'].replace(',',' '),rig['angular_velocity'].replace(',',' '),str(len(bones))]
    for index,bone in enumerate(bones):
        if int(bone['index'])!=index: raise ValueError('Bone order mismatch')
        out.append(bone['name']+' '+bone['parent']+' '+bone['position'].replace(',',' ')+' '+bone['rotation'].replace(',',' '))
    if version>=3:
        out.append(' '.join(str(body_ids[i]) for i in range(len(bones))))
    if version>=4:
        ground=grounds[0];shape=(shape_directory/('ground-'+ground['body_id']+'.jolt')).resolve()
        if not shape.is_file():raise ValueError('Missing ground shape')
        out.append(ground['body_id']+' '+ground['position'].replace(',',' ')+' '+ground['rotation'].replace(',',' ')
                   +' '+ground['friction']+' '+ground['restitution']+' '+json.dumps(str(shape)))
    out.append(str(len(frames)))
    for index in range(warm+len(frames)):
        step=first+index;command=steps[step]
        frame=frames[index-warm] if index>=warm else dict(root=rig['root'],rotation=rig['rotation'])
        if index>=warm and (int(frame['step'])!=step or int(frame['frame'])!=index-warm):
            raise ValueError('Noncontiguous replay frame mapping')
        out.append(str(index)+' '+frame['root'].replace(',',' ')+' '+frame['rotation'].replace(',',' ')
                   +' '+command['filtered'].replace(',',' ')+' '+command['facing_request' if raw_facing else 'facing'].replace(',',' '))
        if version>=2:
            # Consumed base is in the pre-step root frame, not the recorded post-step root.
            previous_rotation=frames[index-warm-1]['rotation'] if index>warm else rig['rotation']
            out.append(command['root'].replace(',',' ')+' '+previous_rotation.replace(',',' '))
            if reconstructed_poses is not None:
                selected=reconstructed_poses.get(step)
                if selected is None or len(selected)!=7*len(bones):
                    raise ValueError('Missing or incomplete reconstructed pose')
            for body in range(len(bones)):
                if reconstructed_poses is not None:
                    out.append(' '.join(str(v) for v in selected[7*body:7*(body+1)]))
                else:
                    pose=poses[(step,body)]
                    out.append(pose['position'].replace(',',' ')+' '+pose['rotation'].replace(',',' '))
            if recovery_poses is not None:
                animation=recovery_poses.get(step)
                if animation is None or len(animation)!=7*len(bones):
                    raise ValueError('Missing or incomplete recovery candidate')
                for body in range(len(bones)):
                    out.append(' '.join(str(v) for v in animation[7*body:7*(body+1)]))
    event_schema=rig.get('recovery_event_schema')
    if event_schema is not None and event_schema not in ('1','2'): raise ValueError('Unknown recovery event schema')
    if recovery_events and event_schema is None: raise ValueError('Recovery events lack schema declaration')
    # Schema1 logger added one to a counter that already names the next solve.
    # Preserve historical readability; schema2 records the zero-based step directly.
    event_step_adjustment=-1 if event_schema=='1' else 0
    recovery_events=[step+event_step_adjustment for step in recovery_events]
    selected_events=[step-first for step in recovery_events if first<=step<first+warm+len(frames)]
    if selected_events!=sorted(set(selected_events)): raise ValueError('Duplicate or unordered recovery events')
    if event_schema is not None:
        out.append('recovery_finish_events_v1 '+str(len(selected_events)))
        out.extend(str(step) for step in selected_events)
    if rig.get('direct_event_schema') is not None:
        if rig['direct_event_schema']!='1' or event_schema is None:
            raise ValueError('Invalid direct event declaration')
        selected_impulses=[]
        for event in direct_impulses:
            step=int(event['next_step'])-first
            if not 0<=step<warm+len(frames): raise ValueError('Direct impulse outside replay coverage')
            if event['point_present'] not in ('0','1'): raise ValueError('Invalid impulse point presence')
            selected_impulses.append(str(step)+' '+event['body']+' '+event['impulse'].replace(',',' ')
                +' '+event['point_present']+' '+event['point'].replace(',',' '))
        out.append('direct_impulses_v1 '+str(len(selected_impulses)))
        out.extend(selected_impulses)
        requests=[step-first for step in recovery_requests]
        if requests!=sorted(set(requests)) or any(not 0<=step<warm+len(frames) for step in requests):
            raise ValueError('Invalid recovery request timeline')
        out.append('recovery_requests_v1 '+str(len(requests)))
        out.extend(str(step) for step in requests)
    elif direct_impulses or recovery_requests:
        raise ValueError('Direct impulse lacks rig declaration')
    if contact_inputs:
        required=('duration','locomotion_requested','interrupted','left_supported','right_supported',
                  'production_ready','measured_velocity','left_slip','right_slip','left_load','right_load',
                  'support_error','incoming_available','incoming_error','requested_direction')
        selected=[]
        selected_steps=set(range(first,first+warm+len(frames)))
        if set(contact_inputs).intersection(selected_steps)!=selected_steps:
            raise ValueError('Incomplete contact-root input timeline')
        for index,step in enumerate(range(first,first+warm+len(frames))):
            item=contact_inputs[step]
            if item.get('present') not in ('0','1'): raise ValueError('Invalid contact-root input presence')
            if item['present']=='0':
                selected.append(str(index)+' 0')
                continue
            if any(name not in item for name in required): raise ValueError('Truncated contact-root input')
            bools=[item[name] for name in ('locomotion_requested','interrupted','left_supported',
                                           'right_supported','production_ready','incoming_available')]
            if any(value not in ('0','1') for value in bools): raise ValueError('Invalid contact-root input boolean')
            scalars=[item[name] for name in ('duration','left_slip','right_slip','left_load','right_load',
                                             'support_error','incoming_error')]
            vectors=[]
            for name in ('measured_velocity','requested_direction'):
                parts=item[name].split(',')
                if len(parts)!=3: raise ValueError('Invalid contact-root input vector')
                vectors.extend(parts)
            try: values=[float(value) for value in scalars+vectors]
            except ValueError as error: raise ValueError('Invalid contact-root input number') from error
            if not all(math.isfinite(value) for value in values) or values[0] <= 0:
                raise ValueError('Non-finite or non-positive contact-root input')
            selected.append(str(index)+' 1 '+item['duration']+' '+' '.join(bools)+' '
                +item['measured_velocity'].replace(',',' ')+' '+item['left_slip']+' '+item['right_slip']+' '
                +item['left_load']+' '+item['right_load']+' '+item['support_error']+' '
                +item['incoming_error']+' '+item['requested_direction'].replace(',',' '))
        out.append('contact_root_inputs_v1 '+str(len(selected)))
        out.extend(selected)
    if contact_cycles:
        if len(contact_cycles)!=1: raise ValueError('Need exactly one contact-root cycle')
        cycle=contact_cycles[0]
        if cycle.get('schema')!='1': raise ValueError('Unknown contact-root cycle schema')
        cycle_frames=int(cycle['frames']); cycle_bodies=int(cycle['bodies'])
        if cycle_frames<2 or cycle_bodies!=len(bones): raise ValueError('Contact-root cycle dimensions mismatch')
        if len(contact_frames)!=cycle_frames or len(contact_poses)!=cycle_frames*cycle_bodies:
            raise ValueError('Incomplete contact-root cycle')
        out.append('contact_root_cycle_v1 '+str(cycle_frames)+' '+str(cycle_bodies)+' '
                   +cycle['left_touchdown']+' '+cycle['right_touchdown']+' '
                   +cycle['stride'].replace(',',' ')+' '+cycle['stride_length']+' '+cycle['phase_rate'])
        for frame in range(cycle_frames):
            item=contact_frames[frame]
            out.append(item['stance']+' '+item['root_delta'].replace(',',' ')+' '+item['vertex'].replace(',',' '))
            for body in range(cycle_bodies):
                item=contact_poses[(frame,body)]
                out.append(item['position'].replace(',',' ')+' '+item['rotation'].replace(',',' '))
    destination.write_text('\n'.join(out)+'\n',encoding='utf-8')
    return dict(pose_source=('render candidates; scenario selects with own mode' if recovery_poses is not None else
                    ('render animation/raw events and observed mode' if reconstructed_poses is not None else 'recorded consumed base')),
                facing_source='native consumed raw request' if raw_facing else 'native filtered output',
                recovery_finish_events_captured=event_schema is not None, recovery_event_step_adjustment=event_step_adjustment, recovery_finish_steps=selected_events,
                actor_kind=actor_kind, frames=len(frames),warmup_frames=warm,creation_step=first,first_capture_step=first+warm,
                ground_source='captured shape, transform, material and body ID' if version>=4 else 'approximate fixture',
                ground_height_assumption='flat-floor-v1 top z=0',
                warmup_pose_assumption=('reconstructed from render history' if reconstructed_poses is not None else
                    ('consumed base pose captured' if version>=2 else 'creation skeleton until first captured animation target')),
                controller_defaults='not stored in context; select matching driver profile or explicit overrides',
                contact_root_cycle=('captured adapted native cycle' if contact_cycles else 'absent'),
                contact_root_inputs=('captured complete native timeline' if contact_inputs else 'absent'))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('log',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--shape-directory',type=Path)
    parser.add_argument('--actor-kind', choices=('player','npc'), default='player')
    parser.add_argument('--reconstruct-render-poses', action='store_true')
    parser.add_argument('--online-pose-selection', action='store_true')
    parser.add_argument('--raw-facing', action='store_true')
    args=parser.parse_args()
    reconstructed=None
    recovery=None
    if args.reconstruct_render_poses or args.online_pose_selection:
        from replay_pose_selection_audit import audit_render_history
        reconstructed=audit_render_history(args.log.read_text(encoding='utf-8').splitlines(),
            args.actor_kind, reconstruct_only=True, paired_candidates=args.online_pose_selection)
        if args.online_pose_selection:
            recovery={step: pair[1] for step,pair in reconstructed.items()}
            reconstructed={step: pair[0] for step,pair in reconstructed.items()}
    print(json.dumps(extract(args.log,args.output,args.shape_directory,args.actor_kind,reconstructed,recovery,args.raw_facing),indent=2))
