# Live uncertainty-aware search (engineering release)

## Question
Can a wall-aware likelihood model and missing-packet handling improve local beacon search compared with a nominal Gaussian model and fixed coverage in a controlled synthetic arena?

## Methods
The new engine uses the same 24 × 16 grid, 0.5 m cells, 90 moves and 0.8 m offline proximity rule as the earlier benchmark. Four layouts are used: the original layout and three connected layouts generated deterministically with seeds 301–303. For each layout, eight seeds (8100–8107) define start/beacon locations and a common sequence of disturbances and packet dropouts. Three RF profiles yield 96 matched cases per policy, 288 runs. Locations may recur. Generated maps are fixed for the benchmark, not continuously changing or unseen-map generalisation evidence.

The sensor model is synthetic log-distance RSSI, with Gaussian noise and a fixed additional attenuation when a ray intersects a blocked grid cell. Profiles: nominal (4 dB standard deviation, 0 dB obstruction penalty, no dropout); shadowed (4 dB noise, 6 dB obstruction penalty, 10% dropout probability); challenging (8 dB noise, 12 dB obstruction penalty, 25% dropout probability). Dropouts are independent in this simplified model. No real-world RF calibration is available. Every method has the same supplied map, exact grid position, motion budget and matched random scenario. Actual measured RSSI depends on the route.

Robust Bayesian search uses a likelihood mixture: for obstructed candidate links it marginalises over 0, 6 and 12 dB offsets, equally weighted; each component combines a narrow Gaussian (80%, sigma 4 dB) and broad Gaussian (20%, sigma 10 dB). Unobstructed links use only the zero-offset mixture. A missing packet leaves the belief unchanged. Nominal Bayesian search uses a single Gaussian with 4 dB sigma and ignores obstruction in the likelihood. Both update log probability for each possible beacon cell, normalise, and use the same MAP-minus-visit-penalty goal selection and A* route planner. The coverage baseline ignores RF and explores in alternating row order. The comparison isolates the robust likelihood design; no new machine-learning model, optimal active sensing, dynamic map building or hardware performance is claimed.

Local belief mass = sum of estimated posterior probabilities within 0.8 m of the MAP cell. Entropy measures posterior uncertainty. Neither metric has been calibrated to physical confirmation confidence. The true beacon is only used by the synthetic sensor and offline scorer, not the policy. The dashboard optionally reveals this truth for evaluation. All policies run the full budget; success measures first entry into the radius and does not cause an online stop. Metrics count grid movements, not real seconds or physical collision safety.

## Results
| Profile | Robust Bayesian | Nominal Bayesian | Coverage |
|---|---:|---:|---:|
| Normal | 30/32 | 32/32 | 16/32 |
| Shadowed | 32/32 | 27/32 | 16/32 |
| Challenging | 30/32 | 28/32 | 16/32 |
| Total | 92/96 | 87/96 | 48/96 |

Robust search improved overall success in this benchmark, but the nominal policy performed better in the normal profile and had fewer mean capped steps in the challenging profile. This is a limited, synthetic sensitivity experiment; do not claim statistical superiority in the general case. Mean final estimate errors were 0.23 m (robust) and 0.51 m (nominal) in this model, not physical accuracy. All 288 runs had zero invalid grid moves. Raw data: `evidence/live_trials.csv`; summary: `evidence/live_summary.csv`.

## Reproduction and live dashboard
From repository root with Python 3.10+:
```
python live_engine.py
python -m unittest -v
python dashboard.py
```
Open http://127.0.0.1:8765. Choose a layout, profile, algorithm and seed; click New experiment, then Step or Run. Reuse identical inputs to compare algorithms. This is a local-only, single-session research tool. The browser invokes the Python policy for each step, unlike the older recorded replay. No dependency installation is needed for these commands. Stop the local server with Ctrl+C.

## Next engineering milestones
Physics-based motion, odometry error, real RF calibration, unknown obstacle maps, online confirmation logic and teacher-supervised hardware validation remain separate milestones. The earlier concept note and video predate this release; do not mix results or claim new capabilities in the formal submission without updating the evidence and having the team review the code. See `HARDWARE.md` for the supervised bench validation plan.
