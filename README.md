# Hybrid SAC–LQR Control for UR3e Multi-Waypoint Motion

Public technical repository accompanying the manuscript:

> A Reinforcement-Learning-Based Hybrid SAC–LQR Framework for UR3e Multi-Waypoint Motion: System Design and Simulation-Based Evaluation

The framework uses a Soft Actor–Critic (SAC) outer loop to generate bounded incremental joint-reference corrections every 50 ms. Six independent per-joint LQR loops track the resulting references at a 5 ms simulation step.

## Evaluation scope

Two evaluation stages are distinguished:

- **Complete MATLAB/Simulink Hybrid SAC–LQR evaluation:** includes the SAC decision layer, joint-reference conversion, six LQR loops, and the simplified per-axis plant.
- **Adapted MATLAB–ROS 2 controller-pipeline evaluation:** uses the trained SAC decision layer, task logic, `ros2_control`, and `use_fake_hardware=true`. It does not execute the Simulink plant or the LQR inner loops.

Neither stage constitutes physical UR3e validation.

## Public repository scope

This repository intentionally contains only the core MATLAB/Simulink implementation, setup notes, demonstration videos, and aggregate evaluation results. It does not include trained agents, checkpoints, raw per-episode outputs, offline reference banks, terminal-assistance configurations, internal robustness-injection code, manuscripts, or reviewer-response files.

## Requirements

- MATLAB R2024b
- Simulink
- Reinforcement Learning Toolbox
- Control System Toolbox
- Deep Learning Toolbox
- ROS Toolbox for the separate ROS 2 workflow

## Repository structure

```text
Hybrid-SAC-LQR-UR3e/
├── matlab/
│   ├── RL_UR3e_Pick_and_Place_Traj.slx
│   ├── TrainSACAgentForPickAndPlaceTraj.m
│   ├── EvaluateAndAnalyzeAgents.m
│   ├── UR3_Model.m
│   ├── SolveForwardKinematics.m
│   ├── LQR_Gains.m
│   ├── Reward.m
│   ├── Observation.m
│   ├── TargetScheduler.m
│   ├── DeltaThetaToCmd.m
│   └── UR3eResetFcn.m
├── results/
│   └── EVALUATION_SUMMARY.md
├── docs/
│   └── setup_notes.md
├── examples/
│   └── run_example.md
├── Videos/
├── README.md
└── LICENSE
```

## Main configuration

- Four phases: PICK, LIFT, CRUISE, PLACE
- Waypoints [mm]: PICK `[-380, -170, 120]`, LIFT `[-380, -170, 180]`, CRUISE `[-290, -110, 265]`, PLACE `[-200, -50, 300]`
- Observation dimension: 27
- Action dimension: 6
- SAC decision period: 50 ms
- LQR/simulation period: 5 ms
- Episode horizon: 15 s
- Terminal phase tolerance: 8 mm
- Strict full-task success criterion: PLACE distance below 5 mm

The training script exposes the actor, twin-critic, replay-buffer, optimiser, and entropy settings reported in the manuscript. The trained policy used for the reported experiments is not distributed.

## Aggregate results

| Evaluation | Full-task result |
|---|---:|
| Nominal Hybrid SAC–LQR | 96% (48/50) |
| Frozen-policy robustness/generalisation tests | 82–100% |
| Matched offline-trajectory LQR | 100% (50/50) |
| Matched offline-trajectory PID | 100% (50/50) |
| Conventional IK–trajectory–LQR | 100% (50/50) |
| ROS 2 controller pipeline, terminal assistance enabled | 96% (48/50) |
| ROS 2 controller pipeline, terminal assistance disabled | 0% (0/50) |

The nominal `4.221 ± 0.381 mm` value is the Cartesian distance recorded at first entry into the strict 5 mm PLACE region for successful episodes. It is not a sustained-placement or settling metric. See [`results/EVALUATION_SUMMARY.md`](results/EVALUATION_SUMMARY.md) for the aggregate tables and scope notes.

## Basic use

```matlab
addpath(genpath(pwd));
UR3_Model;
LQR_Gains;
open_system('RL_UR3e_Pick_and_Place_Traj.slx');
```

To construct and train a new SAC agent using the public configuration:

```matlab
TrainSACAgentForPickAndPlaceTraj
```

Training a new agent will not reproduce the archived policy bit-for-bit because SAC training is stochastic and the reported study retained one completed training run.

## Authors

- Ahmed Iqdymat
- Iulia Stamatescu
- Grigore Stamatescu

Department of Automation and Industrial Informatics, National University of Science and Technology Politehnica of Bucharest.

## License

Copyright (c) 2026 Ahmed Iqdymat. All rights reserved. This repository is
publicly viewable for research inspection and verification, but it is not
open source and no reuse permission is granted. See [`LICENSE`](LICENSE).
