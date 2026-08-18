# Aggregate evaluation results

Only aggregate results reported in the associated manuscript are included here. Raw per-episode files and trained-policy checkpoints are retained privately.

## Nominal MATLAB/Simulink evaluation

| Metric | Result |
|---|---:|
| Evaluation episodes | 50 |
| Strict full-task successes | 48/50 |
| Full-task success rate | 96% |
| Wilson 95% confidence interval | 86.54–98.90% |
| PLACE distance at successful termination | 4.221 ± 0.381 mm |

The PLACE distance is measured when the strict 5 mm terminal condition is first crossed. It is not a steady-state accuracy value.

## Frozen-policy robustness and generalisation

Each condition contains 50 episodes and uses the retained policy without retraining.

| Condition | Successes | Success rate | Wilson 95% CI |
|---|---:|---:|---:|
| Nominal | 48/50 | 96% | 86.54–98.90% |
| Broader initial range (±30°) | 48/50 | 96% | 86.54–98.90% |
| Unseen shifted waypoints | 50/50 | 100% | 92.87–100% |
| Model-parameter perturbation (±10%) | 48/50 | 96% | 86.54–98.90% |
| Observation noise (1% normalised) | 41/50 | 82% | 69.20–90.23% |
| Command delay (50 ms) | 43/50 | 86% | 73.81–93.05% |
| External torque on joint 2 | 47/50 | 94% | 83.78–97.94% |

## Post-entry placement behaviour

The 48 runs that crossed the strict 5 mm PLACE boundary were continued for 2 s.

| Metric | Result |
|---|---:|
| Sustained below 5 mm for 0.5 s | 0/48 |
| Sustained below 5 mm for 1.0 s | 0/48 |
| Sustained below 5 mm for 2.0 s | 0/48 |
| Mean 5 mm retention over 2 s | 14.822 ± 0.352% |
| Mean error over final 0.5 s | 8.606 ± 0.093 mm |
| Mean maximum post-entry position error | 9.002 ± 0.143 mm |
| Mean post-entry Cartesian path length | 21.551 ± 0.853 mm |
| Mean post-entry control effort | 6.095 ± 0.152 V²s |
| Mean end-effector orientation drift over 2 s | 54.693 ± 0.568° |

No absolute end-effector orientation target was specified. The angular value therefore measures drift from first strict PLACE entry, not orientation error relative to a desired pose.

## Conventional baselines

| Method | Full-task success | Termination distance |
|---|---:|---:|
| Hybrid SAC–LQR | 96% (48/50) | 4.221 ± 0.381 mm |
| Matched offline-trajectory LQR | 100% (50/50) | 4.888 ± 0.039 mm |
| Matched offline-trajectory PID | 100% (50/50) | 4.561 ± 0.092 mm |
| Conventional IK–trajectory–LQR | 100% (50/50) | 4.887 ± 0.031 mm |

## Adapted MATLAB–ROS 2 controller pipeline

This separate stage uses `use_fake_hardware=true`, a fixed initial configuration, a 150 ms command publishing period, and a 45 s horizon. It does not execute the Simulink plant or LQR inner loops.

| Metric | Assistance enabled | Assistance disabled |
|---|---:|---:|
| PICK success | 98% (49/50) | 66% (33/50) |
| LIFT success | 98% (49/50) | 64% (32/50) |
| CRUISE success | 96% (48/50) | 32% (16/50) |
| PLACE below 8 mm | 96% (48/50) | 0% (0/50) |
| Strict full-task success | 96% (48/50) | 0% (0/50) |

For successful assisted runs, the settled PLACE distance was `4.82 ± 0.04 mm`. The stored terminal joint configuration has a forward-kinematics PLACE distance of `4.261 mm`; the assisted result therefore characterises the complete assisted protocol rather than the unassisted SAC policy.

| Round-trip latency statistic | Value |
|---|---:|
| Mean | 5.45 ms |
| Median | 5.65 ms |
| Standard deviation | 1.40 ms |
| 95th percentile | 6.01 ms |
| 99th percentile | 6.48 ms |
| Maximum | 25.10 ms |

These results characterise the adapted controller interface and communication pipeline, not physical-robot performance.
