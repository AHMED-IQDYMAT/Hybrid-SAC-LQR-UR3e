function EvaluateAndAnalyzeAgents(runDir, nEval, projectRoot)
%% =========================================================================
%  FILE        : EvaluateAndAnalyzeAgents.m
%  PROJECT     : UR3e RL — Project #1
%  FUNCTION    : Evaluate saved trained agents and export diagnostic results
%
%  AUTHOR      : Ahmed Iqdymat
%  DATE        : 23-Apr-2026 / public cleanup: 20-May-2026
%
%  IMPORTANT
%  -------------------------------------------------------------------------
%  GitHub-ready public version. No local Windows path is hardcoded. The
%  script automatically infers the project root from the location of this
%  file, searches for the original Simulink model outside Runs/, and writes
%  evaluation outputs under the selected run folder.
%
%  Usage examples:
%    EvaluateAndAnalyzeAgents()
%    EvaluateAndAnalyzeAgents(runDir)
%    EvaluateAndAnalyzeAgents(runDir, 50)
%    EvaluateAndAnalyzeAgents(runDir, 50, projectRoot)
%
%  Inputs:
%    runDir       optional run folder containing chk/finalAgent.mat
%    nEval        optional number of evaluation episodes, default = 50
%    projectRoot  optional project root containing the Simulink model and Runs/
%% =========================================================================

SCRIPT_VERSION = "EvaluateScript_Public_v1.0";
MODEL_NAME     = 'RL_UR3e_Pick_and_Place_Traj';

%% Project root and model discovery
if nargin < 3 || isempty(projectRoot)
    startDir = fileparts(mfilename('fullpath'));
    if isempty(startDir)
        startDir = pwd;
    end
    projectRoot = find_project_root(startDir, MODEL_NAME);
end

PROJECT_ROOT = char(projectRoot);
RUNS_ROOT    = fullfile(PROJECT_ROOT, 'Runs');

assert(exist(PROJECT_ROOT, 'dir') == 7, ...
    'Project root not found: %s', PROJECT_ROOT);

addpath(PROJECT_ROOT);
if exist(fullfile(PROJECT_ROOT, 'matlab'), 'dir') == 7
    addpath(fullfile(PROJECT_ROOT, 'matlab'));
end
if exist(fullfile(PROJECT_ROOT, 'scripts'), 'dir') == 7
    addpath(fullfile(PROJECT_ROOT, 'scripts'));
end
if exist(fullfile(PROJECT_ROOT, 'src'), 'dir') == 7
    addpath(fullfile(PROJECT_ROOT, 'src'));
end

allSlx = dir(fullfile(PROJECT_ROOT, '**', [MODEL_NAME '.slx']));
if isempty(allSlx)
    error('Model file %s.slx was not found inside %s', MODEL_NAME, PROJECT_ROOT);
end

allPaths = fullfile({allSlx.folder}, {allSlx.name});
isInRuns = contains(lower(allPaths), lower([filesep 'Runs' filesep]));
candidatePaths = allPaths(~isInRuns);

if isempty(candidatePaths)
    error(['Only copied model snapshots were found inside Runs/. ', ...
           'Place the original %s.slx in the project root or pass projectRoot explicitly.'], MODEL_NAME);
end

MODEL_PATH = candidatePaths{1};
addpath(fileparts(MODEL_PATH));

fprintf('Project root:\n  %s\n', PROJECT_ROOT);
fprintf('Using model file:\n  %s\n\n', MODEL_PATH);

%% Input defaults
if nargin < 2 || isempty(nEval)
    nEval = 50;
end

if nargin < 1 || isempty(runDir)
    assert(exist(RUNS_ROOT, 'dir') == 7, ...
        'Runs root not found: %s. Pass runDir explicitly or create Runs/.', RUNS_ROOT);
    runDir = find_latest_run_dir(RUNS_ROOT);
end

runDir = char(runDir);
assert(exist(runDir, 'dir') == 7, ...
    'runDir does not exist: %s', runDir);

RUN_ID = infer_run_id_from_run_dir(runDir);

%% Paths
chkDir     = fullfile(runDir, 'chk');
evalDir    = fullfile(runDir, 'eval', char(RUN_ID));
evalFigDir = fullfile(evalDir, 'fig');

if ~exist(chkDir, 'dir')
    error('Checkpoint folder not found: %s', chkDir);
end
if ~exist(evalDir, 'dir')
    mkdir(evalDir);
end
if ~exist(evalFigDir, 'dir')
    mkdir(evalFigDir);
end

xlsxPath       = fullfile(evalDir, ['eval_log_' char(RUN_ID) '.xlsx']);
summaryTxtPath = fullfile(evalDir, ['eval_summary_' char(RUN_ID) '.txt']);

fprintf('\n========================================================\n');
fprintf('  Evaluation\n');
fprintf('  RUN_ID  : %s\n', char(RUN_ID));
fprintf('  Run dir : %s\n', runDir);
fprintf('  Eps     : %d per agent\n', nEval);
fprintf('========================================================\n\n');

%% Model and workspace preparation
mdl = MODEL_NAME;

if bdIsLoaded(mdl)
    loadedPath = get_param(mdl, 'FileName');
    if ~strcmpi(loadedPath, MODEL_PATH)
        close_system(mdl, 0);
    end
end

open_system(MODEL_PATH);

UR3_Model;
LQR_Gains;

Ts_sim   = evalin('base', 'Ts_sim');
Ts_agent = evalin('base', 'Ts_agent');
Tf       = evalin('base', 'Tf');
MaxStepsPerEpisode = round(Tf / Ts_agent);

set_param(mdl, 'StopTime', num2str(Tf));
set_param(mdl, 'FastRestart', 'off');

fprintf('Evaluation timing matches training:\n');
fprintf('  Ts_sim   = %.4f s\n', Ts_sim);
fprintf('  Ts_agent = %.4f s\n', Ts_agent);
fprintf('  Tf       = %.2f s\n', Tf);
fprintf('  MaxStepsPerEpisode = %d\n\n', MaxStepsPerEpisode);

PICK   = [-380, -170, 120];
LIFT   = PICK + [0, 0, 60];
CRUISE = [-290, -110, 265];
PLACE  = [-200,  -50, 300];

assignin('base', 'PICK',  PICK);
assignin('base', 'PLACE', PLACE);

tolPick        = 8.0;
tolLift        = 25.0;
tolCruise      = 25.0;
tolPlace       = 8.0;
tolPlaceStrict = 5.0;

%% Discover agents
agents = {};

finalAgentPath = fullfile(chkDir, 'finalAgent.mat');
if exist(finalAgentPath, 'file')
    agents{end+1} = struct('name', 'finalAgent', 'path', finalAgentPath);
end

bestAgentPath = fullfile(chkDir, 'bestAgent.mat');
if exist(bestAgentPath, 'file')
    agents{end+1} = struct('name', 'bestAgent', 'path', bestAgentPath);
end

for k = 1:9
    fp = fullfile(chkDir, sprintf('bestAgent%d.mat', k));
    if exist(fp, 'file')
        agents{end+1} = struct('name', sprintf('bestAgent%d', k), 'path', fp);
    end
end

if isempty(agents)
    error('No agents found in %s', chkDir);
end

fprintf('Found %d agents:\n', numel(agents));
for a = 1:numel(agents)
    fprintf('  %s\n', agents{a}.name);
end
fprintf('\n');

%% Evaluation loop
summaryRows       = {};
detailRows        = {};
bestEpisodeRows   = {};
phaseStepRows     = {};
phaseDurationRows = {};
failureRows       = {};

for a = 1:numel(agents)

    agentName = agents{a}.name;
    agentPath = agents{a}.path;

    fprintf('=== %s ===\n', agentName);

    loaded = load(agentPath);
    fn = fieldnames(loaded);
    agent = loaded.(fn{1});
    assignin('base', 'agent', agent);

    ret    = nan(nEval,1);
    stp    = nan(nEval,1);
    mxp    = nan(nEval,1);
    dst    = nan(nEval,1);

    mpL    = nan(nEval,1);
    mlL    = nan(nEval,1);
    mcL    = nan(nEval,1);
    mplL   = nan(nEval,1);

    fpL    = nan(nEval,1);
    flL    = nan(nEval,1);
    fcL    = nan(nEval,1);
    fplL   = nan(nEval,1);

    fxErrL = nan(nEval,1);
    fyErrL = nan(nEval,1);
    fzErrL = nan(nEval,1);

    netRedL = nan(nEval,1);
    oscL    = nan(nEval,1);

    rp   = false(nEval,1);
    rl   = false(nEval,1);
    rc   = false(nEval,1);
    pz   = false(nEval,1);
    ps   = false(nEval,1);

    for ep = 1:nEval

        rng(1000 + ep);

        simIn = Simulink.SimulationInput(mdl);
        simIn = UR3eResetFcn(simIn);

        try
            simOut = sim(simIn);
        catch ME
            fprintf('  Ep%02d: sim failed — %s\n', ep, ME.message);

            detailRows(end+1,:) = { ...
                agentName, ep, nan, nan, nan, nan, nan, nan, nan, ...
                false, false, false, false, false, false, nan };

            bestEpisodeRows(end+1,:) = { ...
                agentName, ep, nan, nan, nan, nan, nan, nan, nan, 0 };

            phaseStepRows(end+1,:) = { ...
                agentName, ep, nan, nan, nan, nan };

            phaseDurationRows(end+1,:) = { ...
                agentName, ep, nan, nan, nan, nan };

            failureRows(end+1,:) = { ...
                agentName, ep, "SimulationError", nan, nan, ...
                nan, nan, nan, nan, nan, 0 };
            continue;
        end

        [cp, ph, rw, isd, ds, sc, rt] = extract_logs(simOut);

        [mp, ml, mc, mpl] = phase_mindist(cp, ph, PICK, LIFT, CRUISE, PLACE);
        [fp, fl, fc, fpl] = final_distances(cp, PICK, LIFT, CRUISE, PLACE);
        [firstPick, firstLift, firstCruise, firstPlace] = first_phase_steps(ph);
        [dur1, dur2, dur3, dur4] = phase_durations(ph);
        [finalXErr, finalYErr, finalZErr] = final_xyz_error(cp, ph, PICK, LIFT, CRUISE, PLACE);
        [netReduction, oscNearTarget] = distance_diagnostics(cp, ph, PICK, LIFT, CRUISE, PLACE);
        doneReason = infer_done_reason(ph, cp, ds, sc, PLACE, tolPlaceStrict);

        ret(ep)    = rt;
        stp(ep)    = sc;
        mxp(ep)    = max_or_default(ph, 1);
        dst(ep)    = ds;

        mpL(ep)    = mp;
        mlL(ep)    = ml;
        mcL(ep)    = mc;
        mplL(ep)   = mpl;

        fpL(ep)    = fp;
        flL(ep)    = fl;
        fcL(ep)    = fc;
        fplL(ep)   = fpl;

        fxErrL(ep) = finalXErr;
        fyErrL(ep) = finalYErr;
        fzErrL(ep) = finalZErr;

        netRedL(ep) = netReduction;
        oscL(ep)    = oscNearTarget;

        rp(ep) = reached_phase(ph, 1);
        rl(ep) = reached_phase(ph, 2);
        rc(ep) = reached_phase(ph, 3);
        pz(ep) = place_success(cp, PLACE, tolPlace);
        ps(ep) = place_success(cp, PLACE, tolPlaceStrict);

        detailRows(end+1,:) = { ...
            agentName, ep, rt, sc, mxp(ep), mp, ml, mc, mpl, ...
            rp(ep), rl(ep), rc(ep), pz(ep), ps(ep), ...
            (rp(ep) && rl(ep) && rc(ep) && ps(ep)), ds };

        bestEpisodeRows(end+1,:) = { ...
            agentName, ep, rt, sc, mxp(ep), mp, ml, mc, mpl, ...
            double(rp(ep) && rl(ep) && rc(ep) && ps(ep)) };

        phaseStepRows(end+1,:) = { ...
            agentName, ep, firstPick, firstLift, firstCruise, firstPlace };

        phaseDurationRows(end+1,:) = { ...
            agentName, ep, dur1, dur2, dur3, dur4 };

        failureRows(end+1,:) = { ...
            agentName, ep, doneReason, finalXErr, finalYErr, finalZErr, ...
            fp, fl, fc, fpl, double(ps(ep)) };
    end

    meanReward = mean(ret,  'omitnan');
    meanSteps  = mean(stp,  'omitnan');
    meanMaxPh  = mean(mxp,  'omitnan');

    pickRate   = 100 * mean(rp);
    liftRate   = 100 * mean(rl);
    cruiseRate = 100 * mean(rc);
    place8Rate = 100 * mean(pz);
    place5Rate = 100 * mean(ps);
    fullRate   = 100 * mean(rp & rl & rc & ps);

    summaryRows(end+1,:) = { ...
        agentName, nEval, ...
        meanReward, meanSteps, ...
        pickRate, liftRate, cruiseRate, place8Rate, place5Rate, fullRate, ...
        mean(mpL,'omitnan'), mean(mlL,'omitnan'), ...
        mean(mcL,'omitnan'), mean(mplL,'omitnan'), ...
        mean(dst,'omitnan'), meanMaxPh };

    fprintf('  Pick:%.0f%%  Lift:%.0f%%  Cruise:%.0f%%  Place8:%.0f%%  Place5:%.0f%%  Full:%.0f%%\n', ...
        pickRate, liftRate, cruiseRate, place8Rate, place5Rate, fullRate);
    fprintf('  MinDist mean: Pick=%.2f  Lift=%.2f  Cruise=%.2f  Place=%.2f mm\n', ...
        mean(mpL,'omitnan'), mean(mlL,'omitnan'), mean(mcL,'omitnan'), mean(mplL,'omitnan'));

    traj = struct();
    traj.cp  = cp;
    traj.ph  = ph;
    traj.rw  = rw;
    traj.isd = isd;

    try
        save_figures(evalFigDir, agentName, RUN_ID, traj, ...
            PICK, LIFT, CRUISE, PLACE, ...
            rp, rl, rc, pz, ps, mpL, mlL, mcL, mplL);
    catch ME
        warning('Figures for %s failed: %s', agentName, ME.message);
    end

    fprintf('\n');
end

%% Export tables
Tsummary = cell2table(summaryRows, 'VariableNames', ...
    {'AgentName','EvalEpisodes','MeanReward','MeanSteps', ...
     'PickRate_pct','LiftRate_pct','CruiseRate_pct', ...
     'PlaceZone8mmRate_pct','PlaceSuccess5mmRate_pct','FullTaskSuccessRate_pct', ...
     'MeanMinDistPick_mm','MeanMinDistLift_mm','MeanMinDistCruise_mm','MeanMinDistPlace_mm', ...
     'MeanDoneStep','MeanMaxPhase'});

Tdetail = cell2table(detailRows, 'VariableNames', ...
    {'AgentName','Episode','EpisodeReward','EpisodeSteps','MaxPhase', ...
     'MinDistPick_mm','MinDistLift_mm','MinDistCruise_mm','MinDistPlace_mm', ...
     'ReachedPick','ReachedLift','ReachedCruise', ...
     'PlaceZone8mm','PlaceSuccess5mm','FullTaskSuccess','DoneStep'});

Tbest = cell2table(bestEpisodeRows, 'VariableNames', ...
    {'AgentName','Episode','EpisodeReward','EpisodeSteps','MaxPhase', ...
     'MinDistPick_mm','MinDistLift_mm','MinDistCruise_mm','MinDistPlace_mm','FullTaskSuccessFlag'});

Tsteps = cell2table(phaseStepRows, 'VariableNames', ...
    {'AgentName','Episode','FirstPickStep','FirstLiftStep','FirstCruiseStep','FirstPlaceStep'});

Tdur = cell2table(phaseDurationRows, 'VariableNames', ...
    {'AgentName','Episode','Phase1Steps','Phase2Steps','Phase3Steps','Phase4Steps'});

Tfail = cell2table(failureRows, 'VariableNames', ...
    {'AgentName','Episode','DoneReason','FinalXErr_mm','FinalYErr_mm','FinalZErr_mm', ...
     'FinalDistPick_mm','FinalDistLift_mm','FinalDistCruise_mm','FinalDistPlace_mm','PlaceSuccess5mm'});

runInfoKeys = [ ...
    "RUN_ID"; "SCRIPT_VERSION"; "RUN_DIR"; "EVAL_EPISODES"; "MODEL"; "DATE"; ...
    "tolPick_mm"; "tolLift_mm"; "tolCruise_mm"; "tolPlace_mm"; "tolPlaceStrict_mm" ];

runInfoVals = [ ...
    string(RUN_ID); ...
    string(SCRIPT_VERSION); ...
    string(runDir); ...
    string(nEval); ...
    string(MODEL_PATH); ...
    string(datestr(now, 'yyyy-mm-dd HH:MM:SS')); ...
    string(tolPick); ...
    string(tolLift); ...
    string(tolCruise); ...
    string(tolPlace); ...
    string(tolPlaceStrict) ];

Trun = table(runInfoVals, 'RowNames', cellstr(runInfoKeys), 'VariableNames', {'Value'});

writetable(Trun,     xlsxPath, 'Sheet', 'RunInfo', 'WriteRowNames', true);
writetable(Tsummary, xlsxPath, 'Sheet', 'Summary');
writetable(Tdetail,  xlsxPath, 'Sheet', 'PerEpisode');
writetable(Tbest,    xlsxPath, 'Sheet', 'BestEpisodes');
writetable(Tsteps,   xlsxPath, 'Sheet', 'PhaseSteps');
writetable(Tdur,     xlsxPath, 'Sheet', 'PhaseDurations');
writetable(Tfail,    xlsxPath, 'Sheet', 'FailureAnalysis');

write_eval_summary_text(summaryTxtPath, RUN_ID, SCRIPT_VERSION, runDir, nEval, Tsummary);

fprintf('\n=== EVAL COMPLETE ===\n');
fprintf('  %s\n', xlsxPath);
fprintf('  %s\n', summaryTxtPath);
fprintf('  %s\n\n', evalFigDir);

end

%% Helpers
function projectRoot = find_project_root(startDir, modelName)
if nargin < 1 || isempty(startDir)
    startDir = pwd;
end

cur = char(startDir);
while true
    if exist(fullfile(cur, [modelName '.slx']), 'file') == 2 || ...
       ~isempty(dir(fullfile(cur, '**', [modelName '.slx'])))
        projectRoot = cur;
        return;
    end

    parent = fileparts(cur);
    if isempty(parent) || strcmp(parent, cur)
        break;
    end
    cur = parent;
end

error(['Could not infer projectRoot from %s. ', ...
       'Call EvaluateAndAnalyzeAgents(runDir, nEval, projectRoot).'], startDir);
end

function runID = infer_run_id_from_run_dir(runDir)
runID = "Evaluation";
try
    parent1 = fileparts(char(runDir));
    [~, versionName] = fileparts(parent1);
    if strlength(string(versionName)) > 0 && ~strcmpi(versionName, 'Runs')
        runID = string(versionName);
    end
catch
end
end

function runDir = find_latest_run_dir(runsRoot)
dVer = dir(runsRoot);
dVer = dVer([dVer.isdir] & ~startsWith({dVer.name}, '.'));

if isempty(dVer)
    error('No version folders found in %s', runsRoot);
end

[~, ixVer] = sort([dVer.datenum], 'descend');

for i = 1:numel(ixVer)
    verDir = fullfile(runsRoot, dVer(ixVer(i)).name);
    dRun = dir(verDir);
    dRun = dRun([dRun.isdir] & ~startsWith({dRun.name}, '.'));

    if ~isempty(dRun)
        [~, ixRun] = sort([dRun.datenum], 'descend');
        runDir = fullfile(verDir, dRun(ixRun(1)).name);
        return;
    end
end

error('No run folders found inside %s', runsRoot);
end
