# Setup Notes

## MATLAB Environment

The implementation was developed and tested using:

- MATLAB R2024b
- Simulink
- Reinforcement Learning Toolbox
- Control System Toolbox
- Deep Learning Toolbox
- ROS Toolbox

## Simulation Model

The main Simulink model is:

```text
RL_UR3e_Pick_and_Place_Traj.slx
```

## Initialization Order

Before starting the SAC training procedure, run:

```matlab
UR3_Model
LQR_Gains
```

## Training Entry Point

The main training script is:

```matlab
TrainSACAgentForPickAndPlaceTraj
```

## Notes

Large checkpoints, training logs, generated figures, and temporary files are intentionally excluded from the public repository.
