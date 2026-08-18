# Hybrid SAC-LQR Control for UR3e Pick-and-Place

This repository contains the MATLAB/Simulink implementation files for a hybrid Soft Actor-Critic (SAC) and Linear Quadratic Regulator (LQR) control framework applied to a UR3e pick-and-place task.

## Project Overview

The framework combines:

- SAC outer-loop policy for trajectory-level adaptation
- Per-joint LQR inner-loop controllers for joint-level stabilization
- Incremental joint-space action representation
- Four-phase pick-and-place curriculum
- MATLAB/Simulink training environment
- ROS 2 / URSim sim-to-sim validation workflow

## Main Requirements

- MATLAB R2024b
- Simulink
- Reinforcement Learning Toolbox
- Control System Toolbox
- Deep Learning Toolbox
- ROS Toolbox

## Repository Structure

```text
Hybrid-SAC-LQR-UR3e/
│
├── matlab/
│   ├── UR3_Model.m
│   ├── SolveForwardKinematics.m
│   ├── LQR_Gains.m
│   ├── TrainSACAgentForPickAndPlaceTraj.m
│   ├── Reward.m
│   ├── Observation.m
│   ├── TargetScheduler.m
│   ├── DeltaThetaToCmd.m
│   └── UR3eResetFcn.m
│
├── simulink/
│   └── RL_UR3e_Pick_and_Place_Traj.slx
│
├── docs/
│   └── setup_notes.md
│
├── examples/
│   └── run_example.md
│
├── Videos/
│   ├── UR3e_PickPlace_Task.mp4
│   └── Figure_UR3e_Trajectory_Video.mp4
│
├── results/
│   └── UR3e_Training_Evaluation_Logs.xlsx
│
├── README.md
└── LICENSE
```

## Main Files

- `TrainSACAgentForPickAndPlaceTraj.m`: main SAC training script.
- `UR3_Model.m`: UR3e kinematic model and joint limits.
- `LQR_Gains.m`: per-joint LQR gain computation.
- `Reward.m`: reward function used during SAC training.
- `Observation.m`: 27-dimensional observation vector.
- `TargetScheduler.m`: four-phase pick-and-place scheduler.
- `DeltaThetaToCmd.m`: converts incremental actions to absolute joint commands.
- `SolveForwardKinematics.m`: UR3e forward kinematics.
- `UR3eResetFcn.m`: randomized episode reset function.

## Demonstration Videos

### UR3e Pick-and-Place Task

[View Video](Videos/UR3e_PickPlace_Task.mp4)

### UR3e End-Effector Trajectory

[View Video](Videos/Figure_UR3e_Trajectory_Video.mp4)

## Training and Evaluation Logs

The organized training and evaluation logs are provided in:

```text
results/UR3e_Training_Evaluation_Logs.xlsx
```

## Notes

Large training checkpoints, temporary files, generated cache folders, and full raw run directories are not included in this repository.

## Associated Manuscript

Hybrid SAC-LQR Control for High-Precision Pick-and-Place on the UR3e: A Sim-to-Sim Validation Framework.

## Authors

Ahmed Iqdymat  
Supervisor: Dr. Grigore Stamatescu  
University Politehnica of Bucharest  
Faculty of Automatic Control and Computers  
Department of Automation and Industrial Informatics

## License

This repository is provided for academic, research, and educational purposes only. See the `LICENSE` file for details.
