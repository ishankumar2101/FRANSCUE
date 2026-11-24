# FRANSCUE
**St. Francis School | Physical AI Hackathon | Disaster Management (Robotics + AI)**

**Search • Alert • Rescue**

A proposed SOS beacon and AI-assisted search rover for locating distress signals in indoor environments. The runnable research prototype uses synthetic BLE signal measurements, Bayesian location estimation and A* route planning. GPS/SIM updates are part of the planned hardware design.

## Engineering release — live simulation
Run **Python 3.10+** (no third-party packages required):

```sh
python dashboard.py
```

Open **http://127.0.0.1:8765** in a browser. Choose a map, RF profile, seed and policy; start a new experiment and step or run the search. See the live RSSI, dropped packets, posterior heatmap, uncertainty and A* goal. The Python engine computes each step as requested; this is not a prerecorded animation. It is a controlled 2D simulation with a known map and exact grid pose, not a hardware controller or physics-based simulator.

```sh
python live_engine.py
python -m unittest -v
```

### New matched benchmark (288 runs)
| RF condition | Robust Bayesian | Nominal Bayesian | Fixed coverage |
|---|---:|---:|---:|
| Nominal | 30/32 | 32/32 | 16/32 |
| Shadowed | 32/32 | 27/32 | 16/32 |
| Challenging | 30/32 | 28/32 | 16/32 |
| All | **92/96 (95.83%)** | 87/96 (90.62%) | 48/96 (50%) |

Four layouts (three generated), three synthetic RF conditions, 90-move budget, offline 0.8 m approach criterion. Robust likelihood marginalises modelled wall attenuation and uses a broad component for noisy readings. Missing packets leave the posterior unchanged. Nominal search was stronger in the nominal case. These are not field performance measurements. See [full method](docs/LIVE_ENGINE.md), [raw data](evidence/live_trials.csv) and [test log](evidence/all_tests.txt). The existing PDF/video predate these experiments; the team must align formal submission materials and verify original-work requirements before submission.

## Archived benchmark and recorded replay
Download or clone this repository. Open `replay.html` in a browser to see nine recorded simulation cases with a probability heatmap, RSSI, rover path, selected goal and step-by-step controls. No installation is needed for the replay. This is a visual replay of actual computed traces, not a live simulator.

To execute the extended simulation, use Python 3.10 or later (no third-party dependencies):

```sh
python benchmark.py
python -m unittest -v
python make_replay.py
```

The first command creates CSVs and replay data in `evidence/`; the third rebuilds the offline HTML. Run the older twelve-case comparison with `python -m pip install -r requirements.txt` and `python legacy/simulation.py`.

## Extended benchmark results
216 matched trials per method, 648 runs in total. Three indoor grid layouts, three RF conditions, a fixed 90-move budget and an offline 0.8 m proximity criterion.

| Method | Successes | Success rate | Mean capped steps |
|---|---:|---:|---:|
| Bayesian search | 185/216 | 85.65% | 37.51 |
| RSSI exploration | 97/216 | 44.91% | 68.48 |
| Fixed coverage | 96/216 | 44.44% | 62.08 |

| RF condition | Bayesian | RSSI exploration | Fixed coverage |
|---|---:|---:|---:|
| Nominal | 72/72 | 31/72 | 32/72 |
| 6 dB shadowing | 60/72 | 36/72 | 32/72 |
| 8 dB noise and 12 dB shadowing | 53/72 | 30/72 | 32/72 |

All results are simulated. The evaluator scores approach distance after running the policy; a physical stop/confirmation mechanism is not yet implemented. Six automated tests check connectivity, A* routing, sensor model, movement and probability invariants, repeatability and offline scoring. See [method details](docs/METHOD.md) and [raw results](evidence/benchmark_trials.csv).

## Repository contents
- `benchmark.py` — extended reproducible simulation and three search methods.
- `test_benchmark.py` — automated tests.
- `make_replay.py` and `replay.html` — offline recorded-trace visualisation.
- `evidence/` — extended results, traces and test log.
- `legacy/` — original twelve-case simulation and earlier archived data; the extra archived stress-test CSVs do not have their exact runners.
- `firmware/beacon/` — Arduino beacon draft; not compiled or physically tested.
- `docs/Concept_Note.pdf` — existing three-page concept note, predating the extended benchmark.
- `docs/HARDWARE.md` — planned hardware and physical validation.
- `CREDITS.md` — sources and AI assistance acknowledgement.

## Current readiness
The software model is runnable. The beacon firmware is a draft, and full rover firmware, GPS/SIM integration and hardware validation remain development milestones. The project is a supervised research prototype, not a certified emergency device. No real-world localisation performance or physical TRL is claimed from the software alone. Review and understand all materials before submission and verify the event's original-work rules.
