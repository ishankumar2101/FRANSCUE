# Simulation-to-hardware plan

Status: proposed integration. No physical test or firmware compilation is claimed. School-supervised low-voltage bench testing only; never use this prototype as emergency equipment or in an actual hazardous environment.

| Module | Planned component | Evidence to collect |
|---|---|---|
| SOS beacon | ESP32-C3, button, LED, BLE | Activation after continuous 2-second press; scanner observes FRANSCUE-07 |
| Rover control | ESP32-DevKitC V4, TB6612FNG, two DC motors | Verified movement and supervised stop |
| Obstacle sensing | VL53L0X | Measured distance and stop behaviour |
| Pose | Wheel encoders and IMU | Compare pose estimates with marked floor coordinates |
| Search logic | Bayesian estimate, A* on known map | Logged RSSI, map estimate, pose and route |
| Wider-area updates | GPS receiver, SIM-capable cellular modem | Check module compatibility, coverage and successful location update |

GPS/SIM are not part of the current simulation and require a separate integration and network availability. ESP32 alone does not provide cellular service or GPS. The model assumes a known map and accurate pose. For the first supervised demonstration use a marked floor course and a supplied map; validate pose before autonomous navigation. Physical hardware should be assembled and verified with the supervising teacher using board datasheets and suitable power supplies. The beacon draft is not a complete rover controller. No actual disaster deployment is proposed.

## Validation milestones
1. Verify standby, short press, long press, broadcast and reset on the beacon.
2. Measure BLE RSSI at known positions with different obstructions, recording time and distance; fit an RF model on calibration data and evaluate on separate readings.
3. Verify stationary distance readings and pose tracking separately.
4. Integrate supervised motion on a controlled mapped course; retain a reliable manual stop.
5. Compare repeat physical trials with simulation metrics. Report unsuccessful trials as well.

No precise real-world range, localisation accuracy, hardware readiness level, or rescue success rate has been established.
