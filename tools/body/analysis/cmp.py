# cmp.py RUN... : median of key scores for walk and run segments, and total fails.
import json,sys,numpy as np
KEYS=('stepT','stepL','side','gallop','pushoff','both','scuff','sink','lhand')
for run in sys.argv[1:]:
    t=json.load(open(str(__import__('pathlib').Path(__file__).resolve().parents[4] / ('runtime-LOOK-%s/course.json' % run))))
    out=[]
    for kind,segs in (('walk',('walk','walk again')),('run',('run','run again'))):
        rows=[r for r in t if r['segment'] in segs]
        out.append(kind+' '+' '.join('%s %.2f'%(k,np.nanmedian([r[k] for r in rows])) for k in KEYS))
    print('%-8s fails %3d | %s' % (run, sum(len(r['fails']) for r in t), ' || '.join(out)))
