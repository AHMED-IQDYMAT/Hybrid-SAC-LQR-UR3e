function in = UR3eResetFcn(in)
%% =========================================================================
%  FILE        : UR3eResetFcn.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Defines the reset function for randomized initial joint configurations
%  at the beginning of each SAC training episode.
%% =========================================================================

PICK  = [-380, -170, 120];
PLACE = [-200,  -50, 300];

in = setVariable(in, 'PICK',  PICK);
in = setVariable(in, 'PLACE', PLACE);

range_deg      = 20.0;
minSafeDist_mm = 150.0;
maxAttempts    = 100;

theta_init = zeros(1,6);

for attempt = 1:maxAttempts

    theta_try = (rand(1,6) - 0.5) * 2 * deg2rad(range_deg);

    pos_try      = SolveForwardKinematics(theta_try(:));
    dist_to_pick = norm(pos_try(:) - PICK(:));

    if dist_to_pick > minSafeDist_mm
        theta_init = theta_try;
        break
    end
end

in = setVariable(in, 'InitTheta_1', theta_init(1));
in = setVariable(in, 'InitTheta_2', theta_init(2));
in = setVariable(in, 'InitTheta_3', theta_init(3));
in = setVariable(in, 'InitTheta_4', theta_init(4));
in = setVariable(in, 'InitTheta_5', theta_init(5));
in = setVariable(in, 'InitTheta_6', theta_init(6));

end
