"""FRANSCUE Stage-1 CONCEPT SIMULATION (not a hardware-validated model).

A simple grid-world with synthetic BLE RSSI observations and a mapped-obstacle
search rover. The rover maintains a Bayesian position-belief for the beacon;
it does NOT receive the beacon's true coordinates. Its steering uses shortest
safe routes within a known occupancy grid. Limitations: the assumed RSSI model,
known obstacle map and perfect wheel motion are not validated against hardware.

Usage: python simulation.py
Requires: numpy, matplotlib, pillow (for plotting).
"""
import csv
import heapq
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
GRID_W, GRID_H = 24, 16
CELL = 0.5
NOISE_SD = 4.0  # simulation choice: dB
SUCCESS_M = 0.8
MAX_STEPS = 90

# Obstacles in grid coordinates; motion cannot enter these cells.
OBSTACLES = [ (5, 9, 2, 10), (12, 16, 7, 14), (18, 21, 1, 6) ]
occ = np.zeros((GRID_H, GRID_W), dtype=bool)
for x0,x1,y0,y1 in OBSTACLES:
    occ[y0:y1,x0:x1] = True
FREE = [(x,y) for y in range(GRID_H) for x in range(GRID_W) if not occ[y,x]]
FREE_IDX = {p:i for i,p in enumerate(FREE)}
CAND = np.array([[(p[0]+0.5)*CELL,(p[1]+0.5)*CELL] for p in FREE])
START=(2,2)
TEST_TARGETS=[(21,13),(22,2),(3,13),(9,12),(14,3),(20,10),(16,15),(2,14),(10,5),(23,8),(19,15),(7,1)]


def xy(cell):
    return np.array([(cell[0]+.5)*CELL,(cell[1]+.5)*CELL], dtype=float)


def neighbors(cell):
    x,y=cell
    for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
        xx,yy=x+dx,y+dy
        if 0<=xx<GRID_W and 0<=yy<GRID_H and not occ[yy,xx]:
            yield (xx,yy)


def shortest_path(origin,target):
    """A* route on a PRE-KNOWN floor plan, an explicit simulation limitation."""
    if origin == target: return [origin]
    pq=[(0,0,origin)]
    prev={origin:None}
    gscore={origin:0}
    while pq:
        _,g,p=heapq.heappop(pq)
        if p == target:
            path=[]
            while p is not None:
                path.append(p);p=prev[p]
            return list(reversed(path))
        if g!=gscore.get(p): continue
        for n in neighbors(p):
            cost=g+1
            if cost<gscore.get(n,1e9):
                gscore[n]=cost;prev[n]=p
                h=abs(n[0]-target[0])+abs(n[1]-target[1])
                heapq.heappush(pq,(cost+h,cost,n))
    return [origin]


def mean_rssi(rover_xy,candidate_xyz):
    """Synthetic log-distance RF signal; real environments require calibration."""
    dist=np.maximum(np.linalg.norm(candidate_xyz-rover_xy,axis=-1),.35)
    return -53.0-22.0*np.log10(dist)


def probabilistic_trial(beacon, seed=0, record=False, noise_sd=NOISE_SD):
    rng=np.random.default_rng(seed)
    rover=START
    log_belief=np.zeros(len(FREE),dtype=float)
    visits={p:0 for p in FREE}
    history=[]
    for step in range(MAX_STEPS+1):
        dist=float(np.linalg.norm(xy(rover)-xy(beacon)))
        if dist<=SUCCESS_M:
            return dict(success=True,steps=step,history=history,final_dist=dist)
        rssi=float(mean_rssi(xy(rover),xy(beacon))+rng.normal(0,noise_sd))
        expected=mean_rssi(xy(rover),CAND)
        log_belief += -0.5*((rssi-expected)/NOISE_SD)**2
        weights=np.exp(log_belief-log_belief.max())
        weights/=weights.sum()
        visits[rover]+=1
        # Confidence-seeking search: strongest posterior candidate weighted by how
        # many times the rover has already visited that candidate.
        scores=np.log(weights+1e-300)-1.6*np.array([visits[p] for p in FREE])
        # Avoid choosing the current location if possible.
        target=FREE[int(np.argmax(scores))]
        route=shortest_path(rover,target)
        next_cell=route[1] if len(route)>1 else rover
        if next_cell==rover:
            # exploration step: seek an unvisited, reachable neighbour
            reachable=list(neighbors(rover))
            if reachable:
                next_cell=min(reachable,key=lambda p:(visits[p], p[0]+p[1]))
        est_idx=int(np.argmax(weights))
        history.append(dict(step=step,position=rover,rssi=rssi,
                            target=beacon,estimate=FREE[est_idx],
                            confidence=float(weights[est_idx]),
                            entropy=float(-np.sum(weights*np.log(weights+1e-300)))))
        rover=next_cell
    dist=float(np.linalg.norm(xy(rover)-xy(beacon)))
    return dict(success=False,steps=MAX_STEPS,history=history,final_dist=dist)


def coverage_trial(beacon):
    """Deterministic non-BLE baseline: visit unvisited open cells row-by-row."""
    rover=START
    all_cells=sorted(FREE,key=lambda p:(p[1],p[0] if p[1]%2==0 else -p[0]))
    visits={rover}
    target_cursor=0
    for step in range(MAX_STEPS+1):
        if np.linalg.norm(xy(rover)-xy(beacon))<=SUCCESS_M:
            return dict(success=True,steps=step)
        while target_cursor<len(all_cells) and all_cells[target_cursor] in visits:
            target_cursor+=1
        if target_cursor==len(all_cells): break
        target=all_cells[target_cursor]
        route=shortest_path(rover,target)
        rover=route[1] if len(route)>1 else rover
        visits.add(rover)
    return dict(success=False,steps=MAX_STEPS)


def draw_scene(ax,history,upto=None,title=''):
    ax.set_aspect('equal');ax.set_xlim(0,GRID_W*CELL);ax.set_ylim(0,GRID_H*CELL)
    ax.set_facecolor('#f7f8fa')
    for x0,x1,y0,y1 in OBSTACLES:
        ax.add_patch(plt.Rectangle((x0*CELL,y0*CELL),(x1-x0)*CELL,(y1-y0)*CELL,
                                  facecolor='#596477',edgecolor='#354256',linewidth=2))
    shown=history if upto is None else history[:upto+1]
    if shown:
        pts=np.array([xy(p['position']) for p in shown])
        ax.plot(pts[:,0],pts[:,1],color='#1b7197',linewidth=2.1,alpha=.9)
        ax.scatter(*pts[-1],s=95,color='#178aad',edgecolor='white',linewidth=1.4,zorder=5)
        ax.scatter(*xy(shown[-1]['estimate']),marker='X',s=90,color='#ee9625',zorder=4)
        ax.scatter(*xy(shown[-1]['target']),marker='*',s=155,color='#cf3e4d',zorder=4)
    ax.scatter(*xy(START),marker='s',s=95,color='#226b53',label='Rover start',zorder=4)
    ax.set_xticks(np.arange(0,12.1,2)); ax.set_yticks(np.arange(0,8.1,2))
    ax.tick_params(labelsize=8)
    ax.grid(alpha=.13)
    ax.set_xlabel('Position x (m)',fontsize=8);ax.set_ylabel('Position y (m)',fontsize=8)
    ax.set_title(title,fontweight='bold',fontsize=10)


def main():
    results=[]
    for i, beacon in enumerate(TEST_TARGETS):
        probabilistic=probabilistic_trial(beacon, seed=2026+i)
        coverage=coverage_trial(beacon)
        results.append(dict(trial=i+1,beacon_x=xy(beacon)[0],beacon_y=xy(beacon)[1],
                  probabilistic_success=int(probabilistic['success']),probabilistic_steps=probabilistic['steps'],
                  coverage_success=int(coverage['success']),coverage_steps=coverage['steps']))
    with (OUT/'trial_results.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(results[0].keys()));w.writeheader();w.writerows(results)
    for method in ('probabilistic','coverage'):
        successful=[r[f'{method}_steps'] for r in results if r[f'{method}_success']]
        print(f'{method}: {len(successful)}/{len(results)} successes; mean steps among successes {np.mean(successful) if successful else float("nan"):.1f}')
    demo=probabilistic_trial(TEST_TARGETS[0],seed=2026,record=True)
    fig,ax=plt.subplots(figsize=(10,5.4),dpi=145)
    draw_scene(ax,demo['history'],title='FRANSCUE proof-of-concept simulation | blue rover path; red true beacon; orange estimated position')
    fig.tight_layout();fig.savefig(OUT/'simulation_map.png',dpi=165);plt.close(fig)
    return results,demo

if __name__=='__main__':
    main()
