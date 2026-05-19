%% =========================================================================
%  FILE        : UR3_Model.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%                Faculty of Automatic Control and Computers
%                Department of Automation and Industrial Informatics
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Defines the UR3e kinematic model using standard Denavit-Hartenberg
%  parameters and exports the forward-kinematics function and joint limits.
%% =========================================================================

clear global T06                 % Clear previous symbolic transformation
global T06                       % Define global base-to-end-effector matrix

fprintf('\n========================================\n');
fprintf('  UR3_Model.m\n');
fprintf('  UR3e kinematic model initialization\n');
fprintf('========================================\n');

%% -------------------------------------------------------------------------
% 1) Joint limits [rad]
% -------------------------------------------------------------------------

LimitTheta1 = 2*pi;              % Joint 1 software limit: +/- 360 deg
LimitTheta2 = 2*pi;              % Joint 2 software limit: +/- 360 deg
LimitTheta3 = 2*pi;              % Joint 3 software limit: +/- 360 deg
LimitTheta4 = 2*pi;              % Joint 4 software limit: +/- 360 deg
LimitTheta5 = 2*pi;              % Joint 5 software limit: +/- 360 deg
LimitTheta6 = 2*pi;              % Joint 6 software limit: +/- 360 deg

%% -------------------------------------------------------------------------
% 2) Standard Denavit-Hartenberg parameters
% -------------------------------------------------------------------------

syms t1 t2 t3 t4 t5 t6           % Symbolic joint variables [rad]

DH = [ +90,       0,   151.85,  t1 ;   % J1: alpha, a, d, theta
         0,  -243.55,     0,    t2 ;   % J2
         0,  -213.20,     0,    t3 ;   % J3
       +90,       0,   131.05,  t4 ;   % J4
       -90,       0,    85.35,  t5 ;   % J5
         0,       0,    92.10,  t6 ];  % J6

%% -------------------------------------------------------------------------
% 3) Symbolic forward kinematics
% -------------------------------------------------------------------------

T06 = eye(4);                    % Initialize base-to-end-effector transform

for i = 1:6
    alpha_i = deg2rad(DH(i,1));  % Link twist alpha_i [rad]
    a_i     = DH(i,2);           % Link length a_i [mm]
    d_i     = DH(i,3);           % Joint offset d_i [mm]
    theta_i = DH(i,4);           % Joint variable theta_i [rad]

    Ti = [ cos(theta_i), -sin(theta_i)*cos(alpha_i),  sin(theta_i)*sin(alpha_i), a_i*cos(theta_i);
           sin(theta_i),  cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i);
                    0,              sin(alpha_i),              cos(alpha_i),             d_i;
                    0,                         0,                         0,              1 ];

    T06 = T06 * Ti;              % Accumulate transform from frame 0 to frame i
end

%% -------------------------------------------------------------------------
% 4) Numeric forward-kinematics function
% -------------------------------------------------------------------------

UR3e_FK = matlabFunction(T06, ...
    'Vars', {t1, t2, t3, t4, t5, t6});  % Convert symbolic FK to numeric handle

%% -------------------------------------------------------------------------
% 5) Verification at zero joint configuration
% -------------------------------------------------------------------------

T_test   = UR3e_FK(0,0,0,0,0,0);        % Evaluate FK at zero configuration
pos_zero = T_test(1:3,4).';             % Extract Cartesian position [x y z]
expected = [-456.75, -223.15, 66.50];   % Reference zero-pose position [mm]
err_mm   = norm(pos_zero - expected);   % Euclidean verification error [mm]

fprintf('Zero-angle position : [%.2f, %.2f, %.2f] mm\n', pos_zero);
fprintf('Expected position   : [%.2f, %.2f, %.2f] mm\n', expected);

if err_mm < 1.0
    fprintf('Verification         : PASS (error = %.3f mm)\n', err_mm);
else
    fprintf('Verification         : WARNING (error = %.3f mm)\n', err_mm);
end

%% -------------------------------------------------------------------------
% 6) Export variables to MATLAB base workspace
% -------------------------------------------------------------------------

assignin('base','UR3e_FK',     UR3e_FK);       % Export FK function handle
assignin('base','LimitTheta1', LimitTheta1);   % Export joint 1 limit
assignin('base','LimitTheta2', LimitTheta2);   % Export joint 2 limit
assignin('base','LimitTheta3', LimitTheta3);   % Export joint 3 limit
assignin('base','LimitTheta4', LimitTheta4);   % Export joint 4 limit
assignin('base','LimitTheta5', LimitTheta5);   % Export joint 5 limit
assignin('base','LimitTheta6', LimitTheta6);   % Export joint 6 limit

fprintf('Exported variables: UR3e_FK, LimitTheta1..6\n');
fprintf('========================================\n\n');
