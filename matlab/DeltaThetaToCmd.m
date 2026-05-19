function ThetaCmd = DeltaThetaToCmd(DeltaTheta, Theta)
%% =========================================================================
%  FILE        : DeltaThetaToCmd.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Converts the incremental joint-space action produced by the SAC agent
%  into an absolute joint command for the inner-loop LQR controllers.
%% =========================================================================

%#codegen

DeltaTheta = DeltaTheta(:);       % Incremental joint command [rad]
Theta      = Theta(:);            % Current joint angles [rad]

ThetaCmd = Theta + DeltaTheta;    % Absolute joint command [rad]

ThetaCmd = max(-2*pi, ...
           min( 2*pi, ThetaCmd)); % Apply joint software limits

end
