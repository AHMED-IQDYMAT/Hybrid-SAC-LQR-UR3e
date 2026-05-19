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
├── simulink/
│   └── RL_UR3e_Pick_and_Place_Traj.slx
│
├── docs/
│   └── setup_notes.md
│
├── examples/
│   └── run_example.md
│
├── README.md
└── LICENSE
Hybrid-SAC-LQR-UR3e/
│
├── Videos/
│   ├── UR3e_PickPlace_Task.mp4
│   └── Figure_UR3e_Trajectory_Video.mp4
