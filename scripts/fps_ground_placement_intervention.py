"""Translate either the replay actor/reference path or captured floor for geometry controls."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from fps_pose_intervention import layout


def translate(source, output, offset, target='actor'):
    if target not in ('actor','ground'):
        raise ValueError('unknown translation target')
    if output.resolve() == source.resolve() or not math.isfinite(offset):
        raise ValueError('invalid output or offset')
    lines,start,stride,frames = layout(source)
    if offset:
        def move(line, column):
            fields = line.split()
            fields[column] = format(float(fields[column]) + offset, '.17g')
            return ' '.join(fields)
        if target == 'ground':
            # Preserve the quoted snapshot path, including spaces.
            fields = lines[start-2].split(' ',2)
            fields[1] = format(float(fields[1])+offset,'.17g')
            lines[start-2] = ' '.join(fields)
        else:
            lines[2] = move(lines[2],0)
            for step in range(frames):
                index = start + step*stride
                lines[index] = move(lines[index],1)
                lines[index+1] = move(lines[index+1],0)
    output.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    report = {'counterfactual_only':True,'translation_target':target,'offset_x':offset,
              'unchanged':'shape geometry/material, model poses, raw control recording, events, IDs',
              'sha256':{name:hashlib.sha256(path.read_bytes()).hexdigest()
                        for name,path in (('source',source),('output',output))}}
    output.with_suffix('.intervention.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--offset-x',type=float,required=True)
    parser.add_argument('--target',choices=('actor','ground'),default='actor')
    args=parser.parse_args()
    print(json.dumps(translate(args.source,args.output,args.offset_x,args.target),indent=2))
