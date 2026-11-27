"""Map-only fixed-route optimization, independent of beacon test positions."""
import csv, math, random, statistics
from collections import deque
from pathlib import Path
import simulation as s
OUT=Path(__file__).resolve().parent

def plan_route(free, start, neighbors, cell, radius, budget):
    # The planner is passed only geometry. Choose the candidate covering the
    # most free cell centers, breaking ties by earlier coverage across the map.
    bits={p:1<<i for i,p in enumerate(free)}
    footprints={p:sum(bits[q] for q in free if math.dist(p,q)*cell<=radius) for p in free}
    adjacency={p:tuple(neighbors(p)) for p in free}
    rng=random.Random(20261127)
    def candidate():
        adj={p:rng.sample(list(q),len(q)) for p,q in adjacency.items()}
        path=[start]; covered=footprints[start]; area_sum=covered.bit_count()
        while len(path)-1<budget:
            p=path[-1]; remaining=budget-(len(path)-1)
            queue=deque([p]); parent={p:None}; depth={p:0}; masks={p:footprints[p]}
            best=None; bestkey=None
            while queue:
                u=queue.popleft(); d=depth[u]
                if d:
                    gain=(masks[u]&~covered).bit_count()
                    if gain:
                        key=(gain/d,gain,rng.random())
                        if bestkey is None or key>bestkey: best,bestkey=u,key
                if d==remaining: continue
                for v in adj[u]:
                    if v not in parent:
                        parent[v]=u; depth[v]=d+1; masks[v]=masks[u]|footprints[v]; queue.append(v)
            if best is None: break
            extension=[]; u=best
            while u!=p: extension.append(u); u=parent[u]
            for u in reversed(extension):
                path.append(u); covered|=footprints[u]; area_sum+=covered.bit_count()
        area_sum+=(budget+1-len(path))*covered.bit_count()
        return (covered.bit_count(),area_sum),path
    # Fixed planning budget. Test targets are not inputs to candidate selection.
    score,route=max((candidate() for _ in range(64)),key=lambda x:x[0])
    assert len(route)<=budget+1 and all(b in adjacency[a] for a,b in zip(route,route[1:]))
    return route,score[0]

def score(route, beacon):
    hit=next((i for i,p in enumerate(route) if math.dist(p,beacon)*s.CELL<=s.SUCCESS_M),None)
    return int(hit is not None),s.MAX_STEPS if hit is None else hit

def write(name,rows):
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def main():
    route,covered=plan_route(s.FREE,s.START,s.neighbors,s.CELL,s.SUCCESS_M,s.MAX_STEPS)
    rows=[]
    for i,b in enumerate(s.TEST_TARGETS):
        a=s.probabilistic_trial(b,seed=2026+i); old=s.coverage_trial(b); hit,steps=score(route,b)
        rows.append(dict(trial=i+1,beacon_x=float(s.xy(b)[0]),beacon_y=float(s.xy(b)[1]),ai_success=int(a['success']),ai_steps=a['steps'],old_coverage_success=int(old['success']),old_coverage_steps=old['steps'],v3_coverage_success=hit,v3_coverage_steps=steps))
    write('trials.csv',rows)
    write('fixed_route.csv',[dict(step=i,grid_x=p[0],grid_y=p[1],x_m=float(s.xy(p)[0]),y_m=float(s.xy(p)[1])) for i,p in enumerate(route)])
    summary=[]
    for m in ('ai','old_coverage','v3_coverage'):
        n=sum(r[m+'_success'] for r in rows); mean=statistics.mean(r[m+'_steps'] for r in rows)
        summary.append(dict(method=m,trials=len(rows),successes=n,success_percent=round(100*n/len(rows),2),mean_capped_steps=round(mean,4)))
    write('summary.csv',summary)
    map_rows=[dict(grid_x=b[0],grid_y=b[1],success=score(route,b)[0],capped_steps=score(route,b)[1]) for b in s.FREE]
    write('all_free_cells.csv',map_rows)
    assert sum(r['success'] for r in map_rows)==covered
    print('Map coverage:',covered,'/',len(s.FREE),'Moves:',len(route)-1)
    print(*summary,sep='\n')
if __name__=='__main__': main()
