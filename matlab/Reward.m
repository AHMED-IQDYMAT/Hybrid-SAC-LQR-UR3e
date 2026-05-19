function reward = Reward(TargetPosition, CurrentPosition, PlaceTarget, PickTarget)
%% =========================================================================
%  FILE        : Reward.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Computes the reinforcement learning reward for the active waypoint in
%  the four-phase pick-and-place curriculum.
%% =========================================================================

%#codegen

TargetPosition  = TargetPosition(:);
CurrentPosition = CurrentPosition(:);
PLACE           = PlaceTarget(:);
PICK            = PickTarget(:);

persistent prevDist prevTarget

%% -------------------------------------------------------------------------
% 1) Initialize persistent variables
% -------------------------------------------------------------------------

if isempty(prevDist)
    prevDist   = norm(CurrentPosition - TargetPosition) / 1000;
    prevTarget = TargetPosition;
end

%% -------------------------------------------------------------------------
% 2) Reset previous distance if target changes
% -------------------------------------------------------------------------

if norm(prevTarget - TargetPosition) > 1.0
    prevDist   = norm(CurrentPosition - TargetPosition) / 1000;
    prevTarget = TargetPosition;
end

%% -------------------------------------------------------------------------
% 3) Distance-based reward shaping
% -------------------------------------------------------------------------

dist    = norm(CurrentPosition - TargetPosition) / 1000;
dist_mm = dist * 1000;

reward = -0.02;                   % Small step penalty

delta = prevDist - dist;          % Positive when moving toward target

if delta > 0
    reward = reward + delta * 1000;
else
    reward = reward + delta * 200;
end

prevDist = dist;

%% -------------------------------------------------------------------------
% 4) Terminal and intermediate target detection
% -------------------------------------------------------------------------

isAtPick  = norm(TargetPosition - PICK)  < 1.0;
isAtPlace = norm(TargetPosition - PLACE) < 1.0;

%% -------------------------------------------------------------------------
% 5) Waypoint-specific reward bonus
% -------------------------------------------------------------------------

if isAtPick || isAtPlace

    if dist_mm < 8.0
        reward = reward + 50.0 + 500.0 * (1.0 - dist_mm / 8.0);
    end

else

    if dist_mm < 25.0
        reward = reward + 500.0;
    end

end

%% -------------------------------------------------------------------------
% 6) Reward clipping
% -------------------------------------------------------------------------

reward = max(-5.0, min(600.0, reward));

end
