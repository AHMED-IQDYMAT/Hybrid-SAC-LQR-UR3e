# Run Example

This example shows the basic execution order for preparing and starting the UR3e SAC-LQR training environment.

## 1. Open MATLAB

Use MATLAB R2024b or later.

## 2. Add repository folders to the MATLAB path

```matlab
addpath(genpath(pwd));
```

## 3. Initialize the UR3e model and LQR controller

```matlab
UR3_Model
LQR_Gains
```

## 4. Open the Simulink model

```matlab
open_system('RL_UR3e_Pick_and_Place_Traj.slx')
```

## 5. Start training

```matlab
TrainSACAgentForPickAndPlaceTraj
```

## Notes

Large training checkpoints, generated logs, and full training result folders are not included in this repository.
