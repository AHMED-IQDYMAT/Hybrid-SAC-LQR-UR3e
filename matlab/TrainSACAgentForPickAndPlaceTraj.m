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
rng(0);

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
% 6) Create the SAC actor and twin critics
% -------------------------------------------------------------------------

actorLG = layerGraph();
actorLG = addLayers(actorLG, featureInputLayer(numObs,'Name','obs'));
actorLG = addLayers(actorLG, [
    fullyConnectedLayer(512,'Name','a_fc1')
    reluLayer('Name','a_relu1')
    fullyConnectedLayer(256,'Name','a_fc2')
    reluLayer('Name','a_relu2')
    fullyConnectedLayer(128,'Name','a_fc3')
    reluLayer('Name','a_relu3')]);
actorLG = connectLayers(actorLG,'obs','a_fc1');

actorLG = addLayers(actorLG, ...
    fullyConnectedLayer(numAct,'Name','mean'));
actorLG = connectLayers(actorLG,'a_relu3','mean');

actorLG = addLayers(actorLG, [
    fullyConnectedLayer(numAct,'Name','std_fc')
    softplusLayer('Name','std')]);
actorLG = connectLayers(actorLG,'a_relu3','std_fc');

actor = rlContinuousGaussianActor(actorLG,obsInfo,actInfo, ...
    'ObservationInputNames','obs', ...
    'ActionMeanOutputNames','mean', ...
    'ActionStandardDeviationOutputNames','std');

critic1 = rlQValueFunction(buildCritic(numObs,numAct,'c1_'),obsInfo,actInfo, ...
    'ObservationInputNames','obs', ...
    'ActionInputNames','DeltaTheta');

critic2 = rlQValueFunction(buildCritic(numObs,numAct,'c2_'),obsInfo,actInfo, ...
    'ObservationInputNames','obs', ...
    'ActionInputNames','DeltaTheta');

agentOpts = rlSACAgentOptions;
agentOpts.SampleTime             = Ts_agent;
agentOpts.DiscountFactor         = 0.99;
agentOpts.MiniBatchSize          = 256;
agentOpts.ExperienceBufferLength = 3e6;
agentOpts.TargetSmoothFactor     = 1e-3;

agentOpts.ActorOptimizerOptions = rlOptimizerOptions( ...
    LearnRate=3e-4,GradientThreshold=1);
agentOpts.CriticOptimizerOptions(1) = rlOptimizerOptions( ...
    LearnRate=3e-4,GradientThreshold=1);
agentOpts.CriticOptimizerOptions(2) = rlOptimizerOptions( ...
    LearnRate=3e-4,GradientThreshold=1);

agentOpts.EntropyWeightOptions.EntropyWeight = 0.01;
agentOpts.EntropyWeightOptions.TargetEntropy = -numAct;
agentOpts.EntropyWeightOptions.LearnRate      = 1e-5;

agent = rlSACAgent(actor,[critic1 critic2],agentOpts);

%% -------------------------------------------------------------------------
% 7) Training options
% -------------------------------------------------------------------------

trainOpts = rlTrainingOptions( ...
    "MaxEpisodes",3000, ...
    "MaxStepsPerEpisode",MaxSteps, ...
    "ScoreAveragingWindowLength",100, ...
    "StopTrainingCriteria","AverageReward", ...
    "StopTrainingValue",9999, ...
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

function net = buildCritic(numObs,numAct,prefix)

lg = layerGraph();
lg = addLayers(lg,featureInputLayer(numObs,'Name','obs'));
lg = addLayers(lg,featureInputLayer(numAct,'Name','DeltaTheta'));

lg = addLayers(lg,[
    fullyConnectedLayer(512,'Name',[prefix 'o_fc1'])
    reluLayer('Name',[prefix 'o_relu1'])
    fullyConnectedLayer(256,'Name',[prefix 'o_fc2'])]);

lg = addLayers(lg,[
    fullyConnectedLayer(512,'Name',[prefix 'a_fc1'])
    reluLayer('Name',[prefix 'a_relu1'])
    fullyConnectedLayer(256,'Name',[prefix 'a_fc2'])]);

lg = addLayers(lg,additionLayer(2,'Name',[prefix 'add']));
lg = addLayers(lg,[
    reluLayer('Name',[prefix 'relu2'])
    fullyConnectedLayer(128,'Name',[prefix 'fc3'])
    reluLayer('Name',[prefix 'relu3'])
    fullyConnectedLayer(1,'Name',[prefix 'q'])]);

lg = connectLayers(lg,'obs',[prefix 'o_fc1']);
lg = connectLayers(lg,'DeltaTheta',[prefix 'a_fc1']);
lg = connectLayers(lg,[prefix 'o_fc2'],[prefix 'add/in1']);
lg = connectLayers(lg,[prefix 'a_fc2'],[prefix 'add/in2']);
lg = connectLayers(lg,[prefix 'add'],[prefix 'relu2']);

net = dlnetwork(lg);

end
