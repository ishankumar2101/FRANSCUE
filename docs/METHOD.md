# Technical method and reproducibility

The extended benchmark uses a 24 by 16 grid (0.5 m cells) and three fixed, connected layouts. A trial has a fixed starting cell, a hidden target at least 2 m away, an RF model and a random seed. For each layout, 12 start/target scenarios, two independent noise repeats and three signal conditions yield 216 matched scenarios per method (648 method runs). Some target or start cells may recur. The same scenario and per-step noise series are shared between compared methods, not the same route-dependent RSSI values.

## Algorithm
The synthetic signal is -53 - 22 log10(max(distance in metres, 0.35)) plus Gaussian noise; if a sampled line crosses an obstacle, an additional scenario-specific attenuation is applied. Conditions: 4 dB noise/no obstruction penalty; 4 dB noise/6 dB penalty; 8 dB noise/12 dB penalty. The estimator uses the nominal model and assumes 4 dB noise, intentionally leaving shadowing unmodelled.

For each accessible cell i, update log-belief by subtracting half the square of (observed RSSI minus expected RSSI at i) divided by 4 dB. Normalise in log space for numerical stability. Choose a goal using log-belief minus 1.6 times previous visits to that cell. A* plans a shortest path on the supplied map, then the rover advances one cell and repeats. The candidate with highest belief is the estimated location, not a proven location. No machine-learning training or neural network is claimed.

## Comparators and scoring
- Bayesian search: uses the entire measurement history via the probability update.
- RSSI exploration: uses the location with strongest observed reading, then targets nearby unvisited cells; a simple heuristic, not a claim to represent all RSSI algorithms.
- Coverage: alternating row order through free cells, without RF readings.

Every method has the same known map, movement budget (90 grid steps), and A* routing. All run the full budget. True beacon coordinates are only available to the synthetic sensor and offline evaluator, never passed into the policy. Success is first entry within 0.8 m of the true beacon, not an online stop or a confirmed rescue. Mean capped steps includes unsuccessful runs as 90; final estimate error is separate and computed only for methods with estimates. No invalid grid moves is a map-level property, not a physical safety guarantee.

The extended benchmark complements rather than replaces the original concept note's results. Original twelve-trial reproduction uses legacy/simulation.py, which stops using simulator ground truth. The archived extra stress-test CSVs were generated earlier; their exact runner is not available, so those exact records are not independently reproduced here. The new benchmark has its entire runner, deterministic seeds, data and unit tests included.

## Assumptions
Synthetic log-distance RF, static beacon, idealised discrete motion, known map, exact rover pose, no hardware calibration. Three fixed layouts do not establish general performance. No BLE packet-level simulation, GPS/SIM implementation, live confirmation logic, actual obstacle sensing or physical rescue capability is validated. Further tests should address odometry drift, packet dropouts, unseen layouts and physical RF readings.
