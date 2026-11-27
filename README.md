# FRANSCUE — V4 fixed-coverage route optimization

## Reproduce
Python 3.10+; install numpy, matplotlib and pillow. Run `python coverage_v4.py` from this folder. The four generated CSV files are included. Two consecutive executions produced byte-identical CSV files in the test environment.

## Method
The original simulation.py is unchanged. The 24 x 16 grid (12 m x 8 m, 0.5 m cells), three obstacles, start (2,2), twelve original beacon positions, 4 dB Gaussian RSSI noise for Bayesian search, 90-move budget and 0.8 m scoring radius remain unchanged. Coverage is a fixed, map-only route, not an RSSI-informed policy. It initializes from the V3 geometric planner and makes 48,000 stochastic segment-refinement attempts (four rounds of 12,000, seed 46191). Route selection maximizes the number of free cell centers within the proximity threshold and then cumulative area coverage; it does not receive the test beacon coordinates. Final route is 90 adjacent legal moves. Offline first-hit scoring is used; misses count as 90 moves. Geometric proximity does not model occlusion or actual radio detection. Original AI and baseline use proximity stopping in the simulator.

## Results
- Original Bayesian search: 12/12, 41.5833 mean capped moves.
- Original row-by-row coverage: 3/12, 76.25 mean capped moves.
- V4 optimized fixed route: 11/12 (91.67%), 60.25 mean capped moves.
- V4 geometric coverage over every free cell: 273/309 (88.35%).
- Mean capped moves are about 30.98% lower for the Bayesian search than for V4.

This is a new version, not a reconstruction of missing historical evidence. The 12 positions were previously used in development; these are not independent holdout results. The V4 route was selected by geometry, not trial hits. The older presentation's 76.3 steps and 45.48% reduction belong to a different baseline and must not be combined with the V4 success rate. Shadowing/stress tests are not evaluated by this benchmark. No hardware validation is implied. Preserve the old results and label this evaluation V4.

## Files
- `coverage_v4.py`: route optimizer and benchmark entry point.
- `coverage_v3.py`: initial map-only route planner and scoring helpers.
- `simulation.py`: original simulation.
- `trials.csv`: all 12 matched trials.
- `summary.csv`: aggregate metrics.
- `fixed_route.csv`: 91 route coordinates including the starting point.
- `all_free_cells.csv`: exhaustive geometric coverage score for all 309 free cells.
