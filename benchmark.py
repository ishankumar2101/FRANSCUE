"""FRANSCUE controlled grid benchmark. Python 3.10+, standard library only.
A hidden beacon is used exclusively by the synthetic sensor and offline scorer.
All methods run the full budget; no ground-truth-dependent stop decision.
"""
import argparse, csv, json, math, random, heapq, statistics
from pathlib import Path
from functools import lru_cache
from collections import deque

ROOT=Path(__file__).resolve().parent
W,H,CELL=24,16,0.5
BUDGET=90
RADIUS=0.8
LAYOUTS={
 'original':[(5,9,2,10),(12,16,7,14),(18,21,1,6)],
 'corridors':[(4,6,0,12),(10,12,4,16),(16,18,0,11),(20,22,5,14)],
 'rooms':[(3,9,5,7),(12,14,0,9),(8,15,11,13),(18,22,3,10)]
}
CONDITIONS={'nominal':(4,0),'shadowed':(4,6),'high_noise_shadowed':(8,12)}
METHODS=('bayesian','rssi_explore','coverage')

def distance(a,b): return CELL*math.hypot(a[0]-b[0],a[1]-b[1])
def expected(a,b): return -53-22*math.log10(max(distance(a,b),.35))

class World:
    def __init__(self,name):
        self.name=name
        self.blocked=frozenset((x,y) for x0,x1,y0,y1 in LAYOUTS[name] for x in range(x0,x1) for y in range(y0,y1))
        self.free=tuple((x,y) for y in range(H) for x in range(W) if (x,y) not in self.blocked)
        self.valid=frozenset(self.free)
        self.adj={p:tuple(q for q in ((p[0]+1,p[1]),(p[0]-1,p[1]),(p[0],p[1]+1),(p[0],p[1]-1)) if q in self.valid) for p in self.free}
        self.rssi={p:tuple(expected(p,q) for q in self.free) for p in self.free}
    @lru_cache(maxsize=None)
    def route(self,start,goal):
        if start==goal:return (start,)
        g={start:0}; parent={}; queue=[(0,0,start)]
        while queue:
            _,cost,p=heapq.heappop(queue)
            if cost!=g[p]:continue
            if p==goal:
                out=[p]
                while p!=start:p=parent[p];out.append(p)
                return tuple(reversed(out))
            for q in self.adj[p]:
                n=cost+1
                if n<g.get(q,99999):
                    g[q]=n;parent[q]=p
                    heapq.heappush(queue,(n+abs(q[0]-goal[0])+abs(q[1]-goal[1]),n,q))
        return (start,)
    @lru_cache(maxsize=None)
    def shadowed(self,a,b):
        # Approximate line intersection with obstacle cells; one penalty per obstructed link.
        dx,dy=b[0]-a[0],b[1]-a[1]; n=max(1,math.ceil(4*math.hypot(dx,dy)))
        return any((math.floor(a[0]+.5+dx*i/n),math.floor(a[1]+.5+dy*i/n)) in self.blocked for i in range(1,n))

def simulate(world,start,method,observations,record=False):
    """Policy receives only map, start and observation callback, not beacon position."""
    p=start; visits={q:0 for q in world.free}; logp=[0.]*len(world.free)
    order=sorted(world.free,key=lambda q:(q[1],q[0] if q[1]%2==0 else -q[0]))
    seen={}; frames=[]; path=[]; estimates=[]
    for t in range(BUDGET+1):
        path.append(p); visits[p]+=1; signal=observations(p,t) if method!='coverage' else None
        estimate=None; entropy=None; peak=None; belief=None
        if method=='bayesian':
            logp=[a-.5*((signal-m)/4)**2 for a,m in zip(logp,world.rssi[p])]
            top=max(logp);logp=[a-top for a in logp]
            idx=max(range(len(logp)),key=logp.__getitem__);estimate=world.free[idx]
            weights=[math.exp(a) for a in logp]; z=sum(weights)
            peak=1/z;entropy=-sum((w/z)*math.log(w/z) for w in weights if w/z>0)
            if record: belief=[round(w/z,6) for w in weights]
            # MAP estimate plus a repeat-visit penalty (same basic method as legacy).
            goal=world.free[max(range(len(logp)),key=lambda i:logp[i]-1.6*visits[world.free[i]])]
            route=world.route(p,goal)
            nxt=route[1] if len(route)>1 else min(world.adj[p],key=lambda q:(visits[q],q[0]+q[1]))
        elif method=='rssi_explore':
            # Strongest observed location with a neighboring-cell exploration preference.
            seen[p]=signal
            best=max(seen,key=seen.get);estimate=best
            unvisited=[q for q in world.free if not visits[q]]
            goal=min(unvisited,key=lambda q:(distance(best,q)+.35*distance(p,q),q)) if unvisited else best
            route=world.route(p,goal);nxt=route[1] if len(route)>1 else min(world.adj[p],key=lambda q:(visits[q],q))
        elif method=='coverage':
            goal=next((q for q in order if not visits[q]),p)
            route=world.route(p,goal);nxt=route[1] if len(route)>1 else p
        else:raise ValueError(method)
        estimates.append(estimate)
        if record: frames.append({'position':p,'rssi':None if signal is None else round(signal,1),'estimate':estimate,'goal':goal,'entropy':None if entropy is None else round(entropy,3),'peak':None if peak is None else round(peak,4),'belief':belief})
        if t<BUDGET:p=nxt
    return path,estimates,frames

def case(world,start,beacon,noise,atten,seed,method,record=False):
    rng=random.Random(seed); disturbances=[rng.gauss(0,noise) for _ in range(BUDGET+1)]
    def sensor(p,t):return expected(p,beacon)+disturbances[t]-(atten if atten and world.shadowed(p,beacon) else 0)
    path,estimates,frames=simulate(world,start,method,sensor,record)
    d=[distance(p,beacon) for p in path];hit=next((i for i,x in enumerate(d) if x<=RADIUS),None)
    return {'success':int(hit is not None),'first_hit_step':hit,'capped_steps':BUDGET if hit is None else hit,'min_distance_m':round(min(d),3),'final_estimate_error_m':None if estimates[-1] is None else round(distance(estimates[-1],beacon),3),'invalid_moves':sum(b not in world.adj[a] and a!=b for a,b in zip(path,path[1:]))},frames

def run(out):
    out.mkdir(parents=True,exist_ok=True);rows=[];demos=[]
    for k,name in enumerate(LAYOUTS):
        world=World(name); rng=random.Random(6100+k)
        starts=rng.sample(list(world.free),3)
        scenarios=[]
        for i in range(12):
            start=starts[i%3]; beacon=rng.choice([p for p in world.free if distance(start,p)>=2]);scenarios.append((start,beacon))
        for c,(noise,atten) in CONDITIONS.items():
            for i,(start,beacon) in enumerate(scenarios):
                for repeat in range(2):
                    seed=90000+k*1000+i*10+repeat
                    for method in METHODS:
                        rec=i==0 and repeat==0
                        metrics,frames=case(world,start,beacon,noise,atten,seed,method,rec)
                        rows.append(dict(layout=name,condition=c,scenario=i+1,repeat=repeat+1,seed=seed,start_x=start[0],start_y=start[1],beacon_x=beacon[0],beacon_y=beacon[1],noise_db=noise,shadow_db=atten,method=method,**metrics))
                        if rec and method=='bayesian':demos.append({'layout':name,'condition':c,'start':start,'beacon':beacon,'blocked':sorted(world.blocked),'free':world.free,'frames':frames,'metrics':metrics})
    with (out/'benchmark_trials.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    summary=[]
    for condition in ['all',*CONDITIONS]:
        for method in METHODS:
            subset=[r for r in rows if r['method']==method and (condition=='all' or r['condition']==condition)]
            errors=[r['final_estimate_error_m'] for r in subset if r['final_estimate_error_m'] is not None]
            summary.append(dict(condition=condition,method=method,trials=len(subset),successes=sum(r['success'] for r in subset),success_percent=round(100*sum(r['success'] for r in subset)/len(subset),2),mean_capped_steps=round(statistics.mean(r['capped_steps'] for r in subset),2),mean_final_estimate_error_m=round(statistics.mean(errors),2) if errors else None,invalid_moves=sum(r['invalid_moves'] for r in subset)))
    with (out/'benchmark_summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary[0]));writer.writeheader();writer.writerows(summary)
    (out/'replays.json').write_text(json.dumps(demos,separators=(',',':')))
    for r in summary: print(r['condition'],r['method'],f"{r['successes']}/{r['trials']}",f"mean capped steps {r['mean_capped_steps']}")
    return rows,summary

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=ROOT/'evidence');args=ap.parse_args();run(args.out)
