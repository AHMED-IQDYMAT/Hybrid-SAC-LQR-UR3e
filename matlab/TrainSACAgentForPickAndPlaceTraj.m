function TrainSACAgentForPickAndPlaceTraj()
%% =========================================================================
%  FILE        : TrainSACAgentForPickAndPlaceTraj.m
%  PROJECT     : Hybrid SAC-LQR Control for UR3e Pick-and-Place
%
%  AUTHOR      : Ahmed Iqdymat
%  SUPERVISOR  : Dr. Grigore Stamatescu
%  AFFILIATION : University Politehnica of Bucharest
%
%  DESCRIPTION
%  -------------------------------------------------------------------------
%  Main SAC training script for the hybrid UR3e reinforcement learning
%  framework. The SAC policy generates incremental joint-space commands,
%  while the LQR controllers provide inner-loop stabilization.
%% =========================================================================

clc;
close all;

%% -------------------------------------------------------------------------
% 1) Initialize robot model and controller parameters
% -------------------------------------------------------------------------

UR3_Model;                        % Load UR3e kinematic model
LQR_Gains;                        % Compute LQR gains

%% -------------------------------------------------------------------------
% 2) Define task-space targets [mm]
% -------------------------------------------------------------------------

PICK  = [-380, -170, 120];
PLACE = [-200,  -50, 300];

assignin('base','PICK',PICK);
assignin('base','PLACE',PLACE);

%% -------------------------------------------------------------------------
% 3) Load Simulink model
% -------------------------------------------------------------------------

mdl = 'RL_UR3e_Pick_and_Place_Traj';

open_system([mdl '.slx']);

%% -------------------------------------------------------------------------
% 4) Observation and action specifications
% -------------------------------------------------------------------------

numObs = 27;                      % Observation dimension
numAct = 6;                       % Six joint actions

obsInfo = rlNumericSpec([numObs 1], ...
    'LowerLimit',-ones(numObs,1), ...
    'UpperLimit', ones(numObs,1));

DeltaTheta_J13 = pi  * Ts_agent;
DeltaTheta_J46 = 2*pi * Ts_agent;

actLower = [-DeltaTheta_J13;
            -DeltaTheta_J13;
            -DeltaTheta_J13;
            -DeltaTheta_J46;
            -DeltaTheta_J46;
            -DeltaTheta_J46];

actUpper = -actLower;

actInfo = rlNumericSpec([numAct 1], ...
    'LowerLimit',actLower, ...
    'UpperLimit',actUpper);

%% -------------------------------------------------------------------------
% 5) Create Simulink RL environment
% -------------------------------------------------------------------------

env = rlSimulinkEnv( ...
    mdl, ...
    [mdl '/RL Agent'], ...
    obsInfo, ...
    actInfo);

env.ResetFcn = @UR3eResetFcn;

%% -------------------------------------------------------------------------
% 6) Create SAC agent
% -------------------------------------------------------------------------

agentOpts = rlSACAgentOptions;
agentOpts.SampleTime = Ts_agent;
agentOpts.DiscountFactor = 0.99;
agentOpts.MiniBatchSize = 256;

agent = rlSACAgent(obsInfo,actInfo,agentOpts);

%% -------------------------------------------------------------------------
% 7) Training options
% -------------------------------------------------------------------------

trainOpts = rlTrainingOptions( ...
    "MaxEpisodes",3000, ...
    "MaxStepsPerEpisode",MaxSteps, ...
    "ScoreAveragingWindowLength",100, ...
    "Verbose",true, ...
    "Plots","training-progress");

%% -------------------------------------------------------------------------
% 8) Train SAC agent
% -------------------------------------------------------------------------

trainingStats = train(agent,env,trainOpts);

%% -------------------------------------------------------------------------
% 9) Save training results
% -------------------------------------------------------------------------

if ~exist('results','dir')
    mkdir('results');
end

save(fullfile('results','trainedAgent.mat'), ...
    'agent', ...
    'trainingStats');

fprintf('Training completed successfully.\n');

end
