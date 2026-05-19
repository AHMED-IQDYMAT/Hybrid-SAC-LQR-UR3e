function obs = Observation(TargetPosition, CurrentPosition, theta, Omega, NextTarget)
%% =========================================================================
%  FILE        : Observation.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Builds the 27-dimensional observation vector used by the SAC agent.
%% =========================================================================

%#codegen

TargetPosition  = TargetPosition(:);
CurrentPosition = CurrentPosition(:);
NextTarget      = NextTarget(:);

theta = theta(:);
Omega = Omega(:);

%% -------------------------------------------------------------------------
% 1) Cartesian error to active waypoint
% -------------------------------------------------------------------------

error_mm = TargetPosition - CurrentPosition;

pos_near = max(-1, min(1, error_mm / 100));
pos_far  = max(-1, min(1, error_mm / 300));

%% -------------------------------------------------------------------------
% 2) Direction to next waypoint
% -------------------------------------------------------------------------

next_error = NextTarget - CurrentPosition;
next_dir   = max(-1, min(1, next_error / 300));

%% -------------------------------------------------------------------------
% 3) Joint-angle encoding
% -------------------------------------------------------------------------

theta_sin = sin(theta);
theta_cos = cos(theta);

%% -------------------------------------------------------------------------
% 4) Joint-velocity normalization
% -------------------------------------------------------------------------

omega_scaled      = zeros(6,1);

omega_scaled(1:3) = Omega(1:3) / pi;
omega_scaled(4:6) = Omega(4:6) / (2*pi);

omega_scaled = max(-1, min(1, omega_scaled));

%% -------------------------------------------------------------------------
% 5) Final observation vector
% -------------------------------------------------------------------------

obs = [ ...
    pos_near;
    pos_far;
    next_dir;
    theta_sin;
    theta_cos;
    omega_scaled ];

end
