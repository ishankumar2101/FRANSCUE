"""Map-only route optimization with fixed computational budget."""
import csv, math, random, statistics
from collections import deque
from pathlib import Path
import simulation as s
from coverage_v3 import plan_route, score, write

def optimize(free,start,neighbors,cell,radius,budget):
    rng=random.Random(46191)
    free=tuple(free); idx={p:i for i,p in enumerate(free)}; n=len(free)
    adj=[tuple(idx[q] for q in neighbors(p)) for p in free]
    fp=[sum(1<<i for i,q in enumerate(free) if math.dist(p,q)*cell<=radius) for p in free]
    ds=[]
    for u in range(n):
        d=[9999]*n; d[u]=0; q=deque([u])
        while q:
            v=q.popleft()
            for w in adj[v]:
                if d[w]==9999: d[w]=d[v]+1; q.append(w)
        ds.append(d)
    initial,_=plan_route(free,start,neighbors,cell,radius,budget)
    initial=[idx[p] for p in initial]
    while len(initial)<budget+1: initial.append(adj[initial[-1]][0])
    def evaluate(path):
        mask=0; acc=0
        for p in path: mask|=fp[p]; acc+=mask.bit_count()
        return mask.bit_count(),acc
    best=initial[:]; bestscore=evaluate(best)
    for restart in range(4):
        path=best[:]; val=evaluate(path)
        for it in range(12000):
            lo=rng.randrange(budget); hi=min(budget,lo+rng.randint(3,32))
            if rng.random()<.2: hi=budget
            length=hi-lo; endpoint=path[hi] if hi<budget else None
            mask=0
            for p in path[:lo+1]+path[hi+1:]: mask|=fp[p]
            segment=[]; u=path[lo]
            for j in range(length):
                left=length-j-1
                options=[v for v in adj[u] if endpoint is None or ds[endpoint][v]<=left]
                if not options: break
                def weight(v):
                    gain=(fp[v]&~mask).bit_count()
                    return gain + .4*max((fp[w]&~(mask|fp[v])).bit_count() for w in adj[v]) + rng.random()*4.5
                u=max(options,key=weight); segment.append(u); mask|=fp[u]
            if len(segment)!=length or (endpoint is not None and u!=endpoint): continue
            proposal=path[:lo+1]+segment+path[hi+1:]
            pv=evaluate(proposal)
            delta=(pv[0]-val[0])+(pv[1]-val[1])/(100*len(free))
            temp=2.5*(1-it/12000)+.08
            if delta>=0 or rng.random()<math.exp(delta/temp): path,val=proposal,pv
            if pv>bestscore: best,bestscore=proposal,pv
        print('Geometry optimization restart',restart+1,'best coverage',bestscore[0],flush=True)
    route=[free[i] for i in best]
    assert len(route)==budget+1 and all(idx[b] in adj[idx[a]] for a,b in zip(route,route[1:]))
    return route,bestscore[0]

def main():
    route,covered=optimize(s.FREE,s.START,s.neighbors,s.CELL,s.SUCCESS_M,s.MAX_STEPS)
    rows=[]
    for i,b in enumerate(s.TEST_TARGETS):
        a=s.probabilistic_trial(b,seed=2026+i); old=s.coverage_trial(b); hit,steps=score(route,b)
        rows.append(dict(trial=i+1,beacon_x=float(s.xy(b)[0]),beacon_y=float(s.xy(b)[1]),ai_success=int(a['success']),ai_steps=a['steps'],old_coverage_success=int(old['success']),old_coverage_steps=old['steps'],v4_coverage_success=hit,v4_coverage_steps=steps))
    write('trials.csv',rows)
    write('fixed_route.csv',[dict(step=i,grid_x=p[0],grid_y=p[1],x_m=float(s.xy(p)[0]),y_m=float(s.xy(p)[1])) for i,p in enumerate(route)])
    summary=[]
    for m in ('ai','old_coverage','v4_coverage'):
        n=sum(r[m+'_success'] for r in rows); mean=statistics.mean(r[m+'_steps'] for r in rows)
        summary.append(dict(method=m,trials=len(rows),successes=n,success_percent=round(100*n/len(rows),2),mean_capped_steps=round(mean,4)))
    write('summary.csv',summary)
    map_rows=[dict(grid_x=b[0],grid_y=b[1],success=score(route,b)[0],capped_steps=score(route,b)[1]) for b in s.FREE]
    write('all_free_cells.csv',map_rows)
    assert sum(r['success'] for r in map_rows)==covered
    print('Map coverage:',covered,'/',len(s.FREE),'Moves:',len(route)-1)
    print(*summary,sep='\n')
if __name__=='__main__': main()
