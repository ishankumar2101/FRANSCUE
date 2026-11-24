import unittest, math, random
from collections import deque
from benchmark import World, LAYOUTS, simulate, case, distance, BUDGET, RADIUS, expected

class BenchmarkTests(unittest.TestCase):
    def test_connected_maps(self):
        for name in LAYOUTS:
            w=World(name); seen={w.free[0]}; q=deque(seen)
            while q:
                for n in w.adj[q.popleft()]:
                    if n not in seen: seen.add(n);q.append(n)
            self.assertEqual(seen,w.valid)
    def test_astar_shortest(self):
        w=World('original'); start=w.free[0]; goal=w.free[-1]; q=deque([(start,0)]);seen={start}
        while q:
            p,n=q.popleft()
            if p==goal:break
            for v in w.adj[p]:
                if v not in seen:seen.add(v);q.append((v,n+1))
        route=w.route(start,goal); self.assertEqual(len(route)-1,n);self.assertTrue(all(b in w.adj[a] for a,b in zip(route,route[1:])))
    def test_sensor_distance(self): self.assertGreater(expected((0,0),(1,0)),expected((0,0),(10,0)))
    def test_policy_full_budget_and_valid_moves(self):
        w=World('rooms')
        for m in ('bayesian','rssi_explore','coverage'):
            p,e,f=simulate(w,w.free[0],m,lambda p,t:-60,True)
            self.assertEqual(len(p),BUDGET+1);self.assertTrue(all(b in w.adj[a] or b==a for a,b in zip(p,p[1:])))
            if m=='bayesian':self.assertTrue(all(abs(sum(v['belief'])-1)<.001 for v in f))
    def test_repeatability(self):
        w=World('corridors'); a=case(w,w.free[0],w.free[-1],4,6,123,'bayesian');b=case(w,w.free[0],w.free[-1],4,6,123,'bayesian');self.assertEqual(a,b)
    def test_offline_hit_scoring(self):
        w=World('original'); a,f=case(w,w.free[0],w.free[-1],4,0,45,'bayesian',True)
        hits=[i for i,v in enumerate(f) if distance(v['position'],w.free[-1])<=RADIUS]
        self.assertEqual(a['first_hit_step'],hits[0] if hits else None);self.assertEqual(a['success'],int(bool(hits)))
if __name__=='__main__':unittest.main()
