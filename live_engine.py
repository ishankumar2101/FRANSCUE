"""FRANSCUE v3: controlled synthetic RF experiments and live decision engine.
No physical control, no network/GPS simulation. True beacon is withheld from policy.
"""
import math, random, csv, statistics, argparse
from pathlib import Path
from collections import deque
from functools import lru_cache
from benchmark import World, W,H,CELL,BUDGET,RADIUS,distance,expected,LAYOUTS

PROFILES={'nominal':(4,0,0), 'shadowed':(4,6,.10), 'challenging':(8,12,.25)}
POLICIES=('robust','bayesian','coverage')

def connected(free):
    seen={next(iter(free))}; q=deque(seen)
    while q:
        x,y=q.popleft()
        for n in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
            if n in free and n not in seen:seen.add(n);q.append(n)
    return len(seen)==len(free)

def generated_layout(seed):
    rng=random.Random(seed); blocks=set(); all_cells={(x,y) for y in range(H) for x in range(W)}
    for _ in range(8):
        x,y=rng.randrange(1,W-4),rng.randrange(1,H-4); w,h=rng.randrange(2,5),rng.randrange(2,6)
        candidate=blocks|{(a,b) for a in range(x,min(x+w,W-1)) for b in range(y,min(y+h,H-1))}
        if len(candidate)<.35*W*H and connected(all_cells-candidate):blocks=candidate
    return blocks

class Arena(World):
    def __init__(self,name):
        self.name=name
        self.blocked=frozenset(generated_layout(int(name.split('-')[1])) if name.startswith('generated-') else ((x,y) for x0,x1,y0,y1 in LAYOUTS[name] for x in range(x0,x1) for y in range(y0,y1)))
        self.free=tuple((x,y) for y in range(H) for x in range(W) if (x,y) not in self.blocked); self.valid=frozenset(self.free)
        self.adj={p:tuple(q for q in ((p[0]+1,p[1]),(p[0]-1,p[1]),(p[0],p[1]+1),(p[0],p[1]-1)) if q in self.valid) for p in self.free}
        self.rssi={p:tuple(expected(p,q) for q in self.free) for p in self.free}
        self.index={p:i for i,p in enumerate(self.free)}
    @lru_cache(maxsize=2048)
    def shadow_row(self,p):return tuple(self.shadowed(p,q) for q in self.free)

def normlog(a):
    m=max(a); return [x-m for x in a]

def likelihood(signal,mean,obstructed):
    # Mixture marginalizes unknown attenuation; broad component tolerates outliers.
    shifts=(0,6,12) if obstructed else (0,)
    return math.log(max(1e-250,sum(.8*math.exp(-.5*((signal-mean+s)/4)**2)/4 + .2*math.exp(-.5*((signal-mean+s)/10)**2)/10 for s in shifts)/len(shifts)))

class Policy:
    def __init__(self,world,start,kind):
        self.world=world; self.p=start; self.kind=kind; self.logp=[0.]*len(world.free); self.visits={p:0 for p in world.free}; self.order=sorted(world.free,key=lambda q:(q[1],q[0] if q[1]%2==0 else -q[0]));self.received=0
    def observe(self,signal):
        w,p=self.world,self.p;self.visits[p]+=1
        if self.kind!='coverage' and signal is not None:
            self.received+=1
            if self.kind=='robust':
                self.logp=normlog([a+likelihood(signal,m,s) for a,m,s in zip(self.logp,w.rssi[p],w.shadow_row(p))])
            else:self.logp=normlog([a-.5*((signal-m)/4)**2 for a,m in zip(self.logp,w.rssi[p])])
        if self.kind=='coverage':
            self.goal=next((q for q in self.order if not self.visits[q]),p);self.estimate=None;self.belief=None;self.entropy=None;self.mass=None
        else:
            weights=[math.exp(a) for a in self.logp]; z=sum(weights);self.belief=[a/z for a in weights]; i=max(range(len(weights)),key=weights.__getitem__);self.estimate=w.free[i]
            self.entropy=-sum(a*math.log(a) for a in self.belief if a>0)
            self.mass=sum(a for a,q in zip(self.belief,w.free) if distance(q,self.estimate)<=RADIUS)
            # MAP with repeat-visit penalty; robust version uses wider posterior support.
            self.goal=w.free[max(range(len(weights)),key=lambda i:self.logp[i]-1.6*self.visits[w.free[i]])]
        route=w.route(p,self.goal)
        self.next=route[1] if len(route)>1 else min(w.adj[p],key=lambda q:(self.visits[q],q))
    def move(self):self.p=self.next

class Session:
    def __init__(self,world,seed,profile,kind,start=None,beacon=None):
        self.world=world;self.seed=seed; self.profile=profile;self.kind=kind; rng=random.Random(seed)
        self.start=start if start is not None else rng.choice(world.free)
        self.beacon=beacon if beacon is not None else rng.choice([p for p in world.free if distance(p,self.start)>=2]); self.policy=Policy(world,self.start,kind)
        noise,atten,loss=PROFILES[profile];self.atten=atten;self.samples=[(rng.gauss(0,noise),rng.random()<loss) for _ in range(BUDGET+1)]
        self.t=0;self.path=[];self.first_hit=None;self.min_dist=math.inf;self.missing=0;self.signal=None;self.update()
    def update(self):
        p=self.policy.p;noise,drop=self.samples[self.t];self.path.append(p);self.signal=None if drop else expected(p,self.beacon)+noise-(self.atten if self.atten and self.world.shadowed(p,self.beacon) else 0)
        self.missing+=int(drop); self.policy.observe(self.signal if self.kind!='coverage' else None)
        d=distance(p,self.beacon);self.min_dist=min(self.min_dist,d)
        if d<=RADIUS and self.first_hit is None:self.first_hit=self.t
    def step(self):
        if self.t<BUDGET:self.policy.move();self.t+=1;self.update()
    def metrics(self):
        return dict(success=int(self.first_hit is not None),first_hit_step=self.first_hit,capped_steps=self.first_hit if self.first_hit is not None else BUDGET,min_distance_m=round(self.min_dist,3),estimate_error_m=round(distance(self.policy.estimate,self.beacon),3) if self.policy.estimate is not None else None,missing_packets=self.missing,invalid_moves=sum(b not in self.world.adj[a] for a,b in zip(self.path,self.path[1:])))
    def state(self):
        p=self.policy
        return dict(width=W,height=H,cell=CELL,layout=self.world.name,profile=self.profile,method=self.kind,seed=self.seed,blocked=sorted(self.world.blocked),free=self.world.free,beacon=self.beacon,position=p.p,path=self.path,estimate=p.estimate,goal=p.goal,belief=p.belief,entropy=p.entropy,local_mass=p.mass,rssi=self.signal if self.kind!='coverage' else None,step=self.t,budget=BUDGET,received=p.received,missing=self.missing,metrics=self.metrics())

def benchmark(out):
    out.mkdir(parents=True,exist_ok=True); rows=[]
    for layout in ('original','generated-301','generated-302','generated-303'):
        world=Arena(layout)
        for profile in PROFILES:
            for seed in range(8100,8108):
                for method in POLICIES:
                    s=Session(world,seed,profile,method)
                    for _ in range(BUDGET):s.step()
                    rows.append(dict(layout=layout,profile=profile,seed=seed,method=method,start=s.start,beacon=s.beacon,**s.metrics()))
        world.route.cache_clear();world.shadowed.cache_clear();world.shadow_row.cache_clear()
    def write(name,data):
        with (out/name).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    write('live_trials.csv',rows);summary=[]
    for profile in ('all',*PROFILES):
        for m in POLICIES:
            a=[r for r in rows if r['method']==m and (profile=='all' or r['profile']==profile)]; errors=[r['estimate_error_m'] for r in a if r['estimate_error_m'] is not None]
            summary.append(dict(profile=profile,method=m,trials=len(a),successes=sum(r['success'] for r in a),success_percent=round(100*statistics.mean(r['success'] for r in a),2),mean_capped_steps=round(statistics.mean(r['capped_steps'] for r in a),2),mean_final_error_m=round(statistics.mean(errors),2) if errors else None,invalid_moves=sum(r['invalid_moves'] for r in a)))
    write('live_summary.csv',summary)
    for r in summary:print(r)
    return rows,summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=Path(__file__).parent/'evidence');args=p.parse_args();benchmark(args.out)
