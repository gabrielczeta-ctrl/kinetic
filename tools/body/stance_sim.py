"""Replay recorded play mouse input through hand control's stance circle
(build 40) and count what each press becomes: guard change only, wind-up
without a cut, cut, or snap cut. Used to size the circle, the chamber and the
cut from the user's own drags (2026-10-04).

usage: python tools/body/stance_sim.py [R C CUT SNAP]   (from the dev root;
reads runtime-GAME-manual-*/manual-recording/ticks-*.jsonl input events)
"""
import json,glob,math,collections,sys
R,C,CUT,SNAP,PICK=[float(a) for a in sys.argv[1:5]]+[25] if len(sys.argv)>4 else (300,1500,1500,3500,25)
def presses():
    out=[]
    for d in sorted(glob.glob('runtime-GAME-manual-*')):
        ticks={}
        for f in sorted(glob.glob(d+'/manual-recording/ticks-*.jsonl')):
            for l in open(f):
                try: t=json.loads(l)
                except ValueError: continue  # a session cut off mid-line
                if t.get('kind')!='input' or t.get('gui'): continue
                e=ticks.setdefault(t['after_tick'],[0,0,0]); e[0]+=t['dx']; e[1]+=t['dy']; e[2]=max(e[2],1 if t['buttons']&1 else 0)
        if not ticks: continue
        cur=None; prev=0
        for k in range(min(ticks),max(ticks)+1):
            dx,dy,h=ticks.get(k,(0,0,prev))
            if h and not prev: cur=[]
            if h: cur.append((dx,-dy))
            if not h and prev and cur is not None: out.append(cur); cur=None
            prev=h
    return out
def sim(seq,scale):
    px=py=0; vx=vy=0; stage='guard'; ev=[]; pend=0; maxr=0
    for dx,dy in seq:
        dx*=scale; dy*=scale; k=min(1,(1/60)/0.03); vx+=(dx*60-vx)*k; vy+=(dy*60-vy)*k
        if stage=='cut': continue
        r=math.hypot(px,py)
        if r>R:
            ex,ey=px/r,py/r; dr=dx*ex+dy*ey; tx,ty=dx-ex*dr,dy-ey*dr; tl=math.hypot(tx,ty)
            inward=-(vx*ex+vy*ey)
            if dr<0 and tl<=abs(dr): pend+=-dr
            elif dr>0: pend=0
            if inward>CUT and dr<0 and r>R+0.2*C: stage='cut'; ev.append('cut'); continue
            if dr<0 and tl<=abs(dr) and pend<120: pass
            elif tl<=abs(dr): px+=dx; py+=dy
            else:
                need=r+dr-R
                if need<=0: px,py=ex*(r+dr)+tx,ey*(r+dr)+ty
                elif tl<=need: rn=min(r+dr-tl,R+C); px,py=ex*rn,ey*rn
                else:
                    left=tl-need; px,py=ex*R+tx/tl*left,ey*R+ty/tl*left
        else:
            sp=math.hypot(vx,vy)
            if sp>SNAP and r>0.3*R and vx*px+vy*py<0 and stage=='wind': stage='cut'; ev.append('snap'); continue
            px+=dx; py+=dy
        rn=math.hypot(px,py)
        if rn>R+C: px*=(R+C)/rn; py*=(R+C)/rn; rn=R+C
        if not(stage=='guard' and rn<PICK): stage='wind'
        maxr=max(maxr,rn)
        if rn>R and 'commit' not in ev: ev.append('commit')
    return ev,maxr
ps=presses()
print('presses',len(ps))
for scale in [1]:
    c=collections.Counter()
    for p in ps:
        ev,maxr=sim(p,scale)
        key=('snap' if 'snap' in ev else 'cut' if 'cut' in ev else 'commit-no-cut' if 'commit' in ev else 'stance-only')
        c[key]+=1
    print(scale, dict(c))
