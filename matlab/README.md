# MATLAB and Simulink files

This folder contains the public core implementation of the UR3e Hybrid SAC–LQR simulation.

## Main entry points

- `TrainSACAgentForPickAndPlaceTraj.m`: constructs the reported SAC network configuration and starts a new training run.
- `EvaluateAndAnalyzeAgents.m`: evaluates a user-supplied trained agent using the nominal protocol.
- `RL_UR3e_Pick_and_Place_Traj.slx`: complete MATLAB/Simulink Hybrid SAC–LQR model.

## Supporting modules

- `UR3_Model.m`: UR3e kinematics and joint limits.
- `SolveForwardKinematics.m`: standalone forward kinematics.
- `LQR_Gains.m`: per-joint LQR design and Simulink workspace parameters.
- `Reward.m`: waypoint reward calculation.
- `Observation.m`: 27-dimensional observation construction.
- `TargetScheduler.m`: four-phase waypoint scheduler.
- `DeltaThetaToCmd.m`: incremental-action to absolute-reference conversion.
- `UR3eResetFcn.m`: randomised initial joint configuration.

The trained policy, checkpoints, raw run folders, robustness-injection code, conventional-baseline models, and ROS 2 terminal-assistance implementation are not included in the public repository.
