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

%% 1) Define the four-waypoint sequence

targets      = zeros(3,4);
targets(:,1) = Pick;
targets(:,2) = Pick + [0; 0; 60];
targets(:,3) = [-290; -110; 265];
targets(:,4) = Place;

tolerances = [8.0; 25.0; 25.0; 8.0];

%% 2) Initialize the persistent phase state

persistent phase

if isempty(phase)
    phase = 1.0;
end

%% 3) Select the active waypoint and advance completed phases

idx    = max(1, min(4, round(phase)));
target = targets(:,idx);
dist   = norm(CurrentPosition - target);

% Re-evaluate after every transition so all outputs describe the newly
% active phase in the same agent step. The loop also handles the unlikely
% case that one sample lies inside more than one consecutive tolerance.
while phase < 4.0 && dist < tolerances(idx)
    phase  = phase + 1.0;
    idx    = max(1, min(4, round(phase)));
    target = targets(:,idx);
    dist   = norm(CurrentPosition - target);
end

%% 4) Return the active and look-ahead waypoints

TargetPosition = target;
phase_out      = phase;
dist_out       = dist;

nextIdx    = min(4, idx + 1);
NextTarget = targets(:,nextIdx);

end
