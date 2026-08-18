%% =========================================================================
%  FILE        : LQR_Gains.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Computes per-joint Linear Quadratic Regulator (LQR) gains for the UR3e
%  inner-loop controllers used in the hybrid SAC-LQR framework.
%% =========================================================================

fprintf('\n========================================\n');
fprintf('  LQR_Gains.m\n');
fprintf('  UR3e per-joint LQR gain computation\n');
fprintf('========================================\n');

%% -------------------------------------------------------------------------
% 1) Timing parameters
% -------------------------------------------------------------------------

Ts_sim   = 0.005;                 % Simulink fixed-step time [s]
Ts_agent = 0.050;                 % SAC agent decision period [s]
Tf       = 15.0;                  % Episode duration [s]
MaxSteps = round(Tf / Ts_agent);  % Maximum RL steps per episode

%% -------------------------------------------------------------------------
% 2) Motor parameters
% -------------------------------------------------------------------------

GR    = 1/101;                    % Gear ratio
Kt    = 0.0437;                   % Torque constant [N.m/A]
Ke    = 0.0437;                   % Back-EMF constant [V.s/rad]
R     = 0.50;                     % Armature resistance [Ohm]
L     = 0.001;                    % Armature inductance [H]
Im    = 5.1e-5;                   % Rotor inertia [kg.m^2]

V_max = 48.0;                     % Voltage saturation [V]
I_max = V_max / R;                % Current saturation estimate [A]

%% -------------------------------------------------------------------------
% 3) UR3e link inertias
% -------------------------------------------------------------------------

I_link = [ ...
    0.00562;
    0.02173;
    0.00654;
    0.00208;
    0.00225;
    0.00014 ];

J = zeros(6,1);                   % Reflected inertia vector

for j = 1:6
    J(j) = Im / GR^2 + sum(I_link(j:6));
end

%% -------------------------------------------------------------------------
% 4) Empirical damping coefficients
% -------------------------------------------------------------------------

b = [ ...
    0.55;
    0.50;
    0.40;
    0.25;
    0.20;
    0.12 ];

%% -------------------------------------------------------------------------
% 5) LQR weighting matrices
% -------------------------------------------------------------------------

Q = diag([300, 900, 30, 1]);     % State weighting matrix
R_lqr = 0.05;                    % Control weighting scalar

%% -------------------------------------------------------------------------
% 6) Per-joint LQR computation
% -------------------------------------------------------------------------

K_lqr = zeros(6,4);

for j = 1:6

    A = [ ...
        0, -1,       0,        0;
        0,  0,       1,        0;
        0,  0, -b(j)/J(j), Kt/J(j);
        0,  0,   -Ke/L,    -R/L ];

    B = [0; 0; 0; 1/L];

    K_lqr(j,:) = lqr(A, B, Q, R_lqr);
end

%% -------------------------------------------------------------------------
% 7) Export variables to MATLAB base workspace
% -------------------------------------------------------------------------

assignin('base','Ts_sim',   Ts_sim);
assignin('base','Ts_agent', Ts_agent);
assignin('base','Tf',       Tf);
assignin('base','MaxSteps', MaxSteps);

assignin('base','V_max', V_max);
assignin('base','I_max', I_max);

assignin('base','K_lqr', K_lqr);

for j = 1:6
    assignin('base', sprintf('K%d', j), K_lqr(j,:));
end

%% -------------------------------------------------------------------------
% 8) Display summary
% -------------------------------------------------------------------------

disp('Per-joint LQR gains:');
disp(K_lqr);

fprintf('========================================\n\n');
