"""Cut-threshold variants of stance_sim.py (cut speed, inward travel, depth) on
the 10-05/06 play presses (run from the dev root; STANCE_GLOB picks sessions, STANCE_CACHE caches them)."""
import json,glob,math,collections,statistics as st,pickle,os,sys
cache=os.environ.get('STANCE_CACHE','')
def presses(pat):
    out=[]
    for d in sorted(glob.glob(pat)):
        ticks={}
        for f in sorted(glob.glob(d+'/manual-recording/ticks-*.jsonl')):
            for l in open(f):
                if '"input"' not in l: continue
                try: t=json.loads(l)
                except ValueError: continue
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
if cache and os.path.exists(cache): ps=pickle.load(open(cache,'rb'))
else:
    ps=presses(os.environ.get('STANCE_GLOB','runtime-GAME-manual-2026100[56]T*'))
    if cache: pickle.dump(ps,open(cache,'wb'))
def sim(seq,R,C,CUT,SNAP,MINT,DEPTH):
    px=py=0; vx=vy=0; stage='guard'; ev=[]; pend=0; tin=0; info=None; maxr=0; ticks_in=0
    for dx,dy in seq:
        k=min(1,(1/60)/0.03); vx+=(dx*60-vx)*k; vy+=(dy*60-vy)*k
        if stage=='cut': continue
        r=math.hypot(px,py)
        if r>R:
            ex,ey=px/r,py/r; dr=dx*ex+dy*ey; tx,ty=dx-ex*dr,dy-ey*dr; tl=math.hypot(tx,ty)
            inward=-(vx*ex+vy*ey)
            if dr<0: tin+=-dr; ticks_in+=1
            elif dr>0: tin=0; ticks_in=0
            if dr<0 and tl<=abs(dr): pend+=-dr
            elif dr>0: pend=0
            if inward>CUT and dr<0 and r>R+DEPTH*C and tin>=MINT:
                stage='cut'; ev.append('cut'); info=(r,inward,tin,ticks_in,maxr); continue
            if dr<0 and tl<=abs(dr) and pend<120: pass
            elif dr<0 and tl<=abs(dr) and pend-(-dr)<120:
                rn=max(r-pend,0); px,py=ex*rn,ey*rn
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
        maxr=max(maxr,rn)
        if not(stage=='guard' and rn<25): stage='wind'
        if rn>R and 'commit' not in ev: ev.append('commit')
    return ev,info
print('presses',len(ps))
for CUT,MINT,DEPTH in [(1500,0,.2),(2500,0,.2),(3500,0,.2),(1500,100,.2),(1500,200,.2),(2500,150,.2),(3000,200,.3),(1500,0,.4)]:
    c=collections.Counter(); infos=[]
    for p in ps:
        ev,info=sim(p,300,1500,CUT,3500,MINT,DEPTH)
        key=('snap' if 'snap' in ev else 'cut' if 'cut' in ev else 'commit-no-cut' if 'commit' in ev else 'stance-only')
        c[key]+=1
        if info: infos.append(info)
    tin=sorted(i[2] for i in infos); tk=sorted(i[3] for i in infos)
    print(f'CUT {CUT} MINT {MINT} DEPTH {DEPTH}: {dict(c)} | inward travel at cut med {st.median(tin):.0f} p10 {tin[len(tin)//10]:.0f} | ticks {st.median(tk):.0f} p10 {tk[len(tk)//10]}')
