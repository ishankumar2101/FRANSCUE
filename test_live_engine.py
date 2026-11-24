import unittest
from live_engine import Arena,Session,Policy,generated_layout,connected,POLICIES,PROFILES,BUDGET,RADIUS,distance

class LiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.w=Arena('generated-301')
    def test_generation(self):
        self.assertEqual(generated_layout(301),generated_layout(301));self.assertNotEqual(generated_layout(301),generated_layout(302));self.assertTrue(connected(self.w.valid))
    def test_missing_measurement_retains_prior(self):
        p=Policy(self.w,self.w.free[0],'robust');p.observe(-60);before=p.logp[:];p.observe(None);self.assertEqual(before,p.logp);self.assertEqual(p.received,1)
    def test_normalisation(self):
        s=Session(self.w,8100,'challenging','robust');
        for _ in range(15):
            self.assertAlmostEqual(sum(s.policy.belief),1);self.assertTrue(0<=s.policy.mass<=1.000000001);s.step()
    def test_matched_scenarios(self):
        ss=[Session(self.w,8100,'challenging',m) for m in POLICIES];self.assertTrue(all((s.start,s.beacon,s.samples)==(ss[0].start,ss[0].beacon,ss[0].samples) for s in ss))
    def test_valid_moves_and_scoring(self):
        for m in POLICIES:
            s=Session(self.w,8100,'challenging',m)
            for _ in range(BUDGET+5):s.step()
            self.assertEqual(s.t,BUDGET);self.assertEqual(s.metrics()['invalid_moves'],0)
            hits=[i for i,p in enumerate(s.path) if distance(p,s.beacon)<=RADIUS];self.assertEqual(s.first_hit,hits[0] if hits else None)
    def test_policy_repeatable(self):
        a=Session(self.w,8102,'shadowed','robust');b=Session(self.w,8102,'shadowed','robust')
        for _ in range(12):a.step();b.step()
        self.assertEqual(a.state(),b.state())
if __name__=='__main__':unittest.main()
