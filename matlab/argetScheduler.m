function [TargetPosition, phase_out, dist_out, NextTarget] = TargetScheduler(Pick, Place, CurrentPosition)
%% =========================================================================
%  FILE        : TargetScheduler.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Implements the four-phase pick-and-place curriculum used by the SAC agent.
%% =========================================================================

%#codegen

Pick            = Pick(:);
Place           = Place(:);
CurrentPosition = CurrentPosition(:);

persistent phase

if isempty(phase)
    phase = 1;
end

%% 1) Define waypoints

PICK   = Pick;
LIFT   = Pick + [0; 0; 60];
CRUISE = [-290; -110; 265];
PLACE  = Place;

%% 2) Select active and next waypoint

switch phase
    case 1
        TargetPosition = PICK;
        NextTarget     = LIFT;

    case 2
        TargetPosition = LIFT;
        NextTarget     = CRUISE;

    case 3
        TargetPosition = CRUISE;
        NextTarget     = PLACE;

    otherwise
        TargetPosition = PLACE;
        NextTarget     = PLACE;
end

%% 3) Compute active distance

dist_out = norm(CurrentPosition - TargetPosition);

%% 4) Phase transition tolerance

if phase == 1 || phase == 4
    tol_mm = 8.0;
else
    tol_mm = 25.0;
end

%% 5) Advance phase

if dist_out < tol_mm && phase < 4
    phase = phase + 1;
end

phase_out = phase;

end
