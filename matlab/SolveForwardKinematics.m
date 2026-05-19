function pos = SolveForwardKinematics(theta)
%% =========================================================================
%  FILE        : SolveForwardKinematics.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Computes the Cartesian end-effector position of the UR3e robot using
%  standard Denavit-Hartenberg forward kinematics.
%% =========================================================================

%#codegen

theta = theta(:);                 % Ensure column-vector format [6x1]

alpha = [ pi/2 ; 0 ; 0 ; pi/2 ; -pi/2 ; 0 ];       % Link twists [rad]
a_dh  = [ 0 ; -243.55 ; -213.20 ; 0 ; 0 ; 0 ];     % Link lengths [mm]
d_dh  = [151.85 ; 0 ; 0 ; 131.05 ; 85.35 ; 92.10]; % Joint offsets [mm]

T = eye(4);                       % Initialize homogeneous transform

for i = 1:6
    ct = cos(theta(i));           % cos(theta_i)
    st = sin(theta(i));           % sin(theta_i)
    ca = cos(alpha(i));           % cos(alpha_i)
    sa = sin(alpha(i));           % sin(alpha_i)

    ai = a_dh(i);                 % Link length [mm]
    di = d_dh(i);                 % Joint offset [mm]

    Ti = [ ct, -st*ca,  st*sa,  ai*ct ;
           st,  ct*ca, -ct*sa,  ai*st ;
            0,     sa,     ca,     di ;
            0,      0,      0,      1 ];

    T = T * Ti;                   % Update base-to-current-frame transform
end

pos = T(1:3,4);                   % End-effector position [x; y; z] in mm

end
