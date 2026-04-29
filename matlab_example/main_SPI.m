%% Sensor Placement Interpolation (SPI) for 5-Story Frame Structure
% ------------------------------------------------------------
% This script implements the Sensor Placement Interpolation (SPI) scheme
% for robust triaxial accelerometer placement under parametric variation.
%
% Main procedure:
%   1) Load candidate sensor nodes and DOFs (translations only)
%   2) Load scenario-dependent mode shapes (100%, 60%, 20%)
%   3) Select the 1st sensor node via interpolated ||Q_3i|| importance metric
%   4) Construct an initial sensor set until Q0 becomes full rank
%   5) Complete Forward Sequential Sensor Placement (FSSP) using EFI3+
%   6) Save runtimes and final selected sensor nodes
%
% Author: Chanwoo Lee
% Repository: https://github.com/cwlee16532/sensor_placement_interpolation_for_parameter_variation
% ------------------------------------------------------------

clear; clc;

%% -------------------------
% User-defined file paths
% -------------------------
% These file/variable definitions are inherited from the original script.
% Put the required .mat files in the same folder as this script, or replace
% the file names below with relative/absolute paths.

fileCandidates = 'sensor_candidate_5f.mat';   % contains sensor_candidate_5f
fileDofs       = 'dofs_5f.mat';               % contains dofs_5f

% Scenario-dependent mode-shape files.
% Each file should contain a mode-shape matrix. The helper function at the
% end searches for the variable names listed in phiVariableNames.
fileScenario = {
    'phi100.mat';   % 100% stiffness scenario
    'phi60.mat';    % 60% stiffness scenario
    'phi20.mat'     % 20% stiffness scenario
};

% Candidate variable names for mode-shape matrices in the .mat files.
% For system5f_*.mat, the expected variable is usually 'phi'.
% For older files such as system_100.mat, variables may be 'phi_100', etc.
phiVariableNames = {
    {'phi', 'phi_100'};
    {'phi', 'phi_60'};
    {'phi', 'phi_20'}
};

%% -------------------------
% Inputs
% -------------------------
% Target modes for each scenario.
% Scenario order: {1}=100%, {2}=60%, {3}=20%
targetModes = cell(1,3);
targetModes{1} = [1, 2, 4, 5];
targetModes{2} = [1, 2, 4, 5];
targetModes{3} = 1:5;

% Scenario weights. The original code used w{1}=w{2}=w{3}=1.
% Here, weights are normalized for interpolation.
w_input = [1, 1, 1];
w = w_input / sum(w_input);

% Final number of triaxial sensors
sensor_n = 6;

% Optional RoI setting.
% If applyRoI = true, redundant candidate nodes are removed before each
% new sensor selection from the 2nd sensor onward. This follows the
% commented RoI block in the original script.
applyRoI = false; % true / false
R_th = 0.2;

% Numerical tolerance used for rank-deficient information matrix
rankTol = 1e-10;

%% -------------------------
% Load Candidate Nodes and DOFs
% -------------------------
S = load(fileCandidates, 'sensor_candidate_5f');
candidateNodeList = S.sensor_candidate_5f;

D = load(fileDofs, 'dofs_5f');
dofs = D.dofs_5f;

% Extract DOFs corresponding to candidate nodes
sensor_index = find(ismember(dofs(:,1), candidateNodeList));
sensor_dofs  = dofs(sensor_index,:);

% Remove rotational DOFs; keep only translational DOFs [1 2 3]
rot_dofs_index = find(ismember(sensor_dofs(:,2), [4 5 6]));
sensor_dofs(rot_dofs_index,:) = [];

% Unique candidate sensor nodes on a triaxial-node basis
sensor_node = unique(sensor_dofs(:,1));

%% -------------------------
% Load Scenario-Dependent Mode Shapes
% -------------------------
phi_s = cell(1, numel(fileScenario));

for s = 1:numel(fileScenario)
    data = load(fileScenario{s});

    % Load mode-shape matrix using allowed variable names
    phi_full = getFirstAvailableVariable(data, phiVariableNames{s}, fileScenario{s});

    % Extract candidate DOF rows and target mode columns
    phi_tmp = phi_full(sensor_index, targetModes{s});

    % Remove rotational DOF rows
    phi_tmp(rot_dofs_index,:) = [];

    phi_s{s} = phi_tmp;
end

% Maximum number of target modes across scenarios
target_mode_n = cellfun(@(x) size(x,2), phi_s);
target_mode_max_n = max(target_mode_n);

%% =========================================================
% STEP 1) Select the 1st Sensor Node (Interpolated ||Q_3i||)
% =========================================================
tic;

sr = zeros(length(sensor_node), numel(phi_s));

for s = 1:numel(phi_s)
    h = waitbar(0, sprintf('Searching 1st node (scenario %d/%d)...', s, numel(phi_s)));

    for i = 1:length(sensor_node)
        node_i_index = find(ismember(sensor_dofs(:,1), sensor_node(i)));
        phi_3i = phi_s{s}(node_i_index,:);
        Q_3i = phi_3i' * phi_3i;
        sr(i,s) = max(abs(eig(Q_3i)));

        waitbar(i/length(sensor_node), h);
    end

    close(h);

    % Normalize importance values within each scenario
    sr(:,s) = normalizeMinMax(sr(:,s));
end

runtime_1st = toc;

% Scenario-weighted interpolation
sr_interp = sr * w(:);

% Select the first node
[~, first_node_index] = max(sr_interp);
selected_node = sensor_node(first_node_index).';

% Remove selected node from candidate pool
candidate_nodes = sensor_node;
candidate_nodes(candidate_nodes == selected_node) = [];

%% =========================================================
% STEP 2) Initial Placement Until Q0 is Full Rank
% =========================================================
tic;

k = 1;
Q_rank_max = 0;
rank_data = [];
removed_nodes = {};
R_ij_rowsum = {};

while Q_rank_max < target_mode_max_n

    % Optional RoI-based redundant candidate removal.
    % This is applied before selecting the next sensor; therefore, after
    % the 1st node has been selected, it affects the 2nd sensor placement.
    if applyRoI
        minRemainingCandidates = sensor_n - k;
        [candidate_nodes, removed_now, R_now] = removeRedundantNodesByRoI( ...
            candidate_nodes, selected_node, sensor_dofs, phi_s, w, R_th, minRemainingCandidates);
        removed_nodes{k+1} = removed_now; 
        R_ij_rowsum{k} = R_now; 
    end

    EfI3_norm = zeros(length(candidate_nodes), numel(phi_s));

    for s = 1:numel(phi_s)
        h = waitbar(0, sprintf('Initial placement (scenario %d/%d)...', s, numel(phi_s)));

        % Current sensor set information matrix Q0
        sel_idx = find(ismember(sensor_dofs(:,1), selected_node));
        Q0 = phi_s{s}(sel_idx,:)' * phi_s{s}(sel_idx,:);

        % Rank of Q0
        Q_rank = rank(Q0, rankTol);
        rank_data(k,s) = Q_rank; 

        % Null-space projector for rank-deficient case
        [psi, ev] = eig(Q0);
        zero_idx = find(diag(ev) < rankTol);

        if isempty(zero_idx)
            P = eye(size(Q0,1));
        else
            P = psi(:,zero_idx) * psi(:,zero_idx)';
        end

        % Candidate information orthogonal to current sensors
        cand_idx = find(ismember(sensor_dofs(:,1), candidate_nodes));
        Qc = phi_s{s}(cand_idx,:)' * phi_s{s}(cand_idx,:);
        Qbar = P * Qc * P';

        % Compute EFI3 metric for each candidate node
        EfI3 = zeros(length(candidate_nodes),1);

        for j = 1:length(candidate_nodes)
            node_j_index = find(ismember(sensor_dofs(:,1), candidate_nodes(j)));
            phi_3j = phi_s{s}(node_j_index,:);

            EfI3(j) = 1 - det(eye(3) - phi_3j * pinv(Qbar) * phi_3j');

            waitbar(j/length(candidate_nodes), h);
        end

        close(h);

        % Normalize EFI3 values
        EfI3_norm(:,s) = normalizeMinMax(EfI3);
    end

    % Scenario-weighted interpolation
    EfI3_interp = EfI3_norm * w(:);

    % Select best node
    [~, max_index] = max(EfI3_interp);
    selected_node = [selected_node candidate_nodes(max_index)]; %#ok<AGROW>
    k = k + 1;

    % Remove selected node from candidate pool
    candidate_nodes(max_index) = [];

    % Update rank criterion across scenarios
    Q_rank_max = 0;
    sel_idx = find(ismember(sensor_dofs(:,1), selected_node));

    for s = 1:numel(phi_s)
        Q0 = phi_s{s}(sel_idx,:)' * phi_s{s}(sel_idx,:);
        Q_rank_max = max(Q_rank_max, rank(Q0, rankTol));
        rank_data(k,s) = rank(Q0, rankTol); 
    end
end

runtime_Initial = toc;

%% =========================================================
% STEP 3) Final Placement Using EFI3+ (FSSP)
% =========================================================
tic;

while k < sensor_n

    % Optional RoI-based redundant candidate removal before EFI3+ selection.
    if applyRoI
        minRemainingCandidates = sensor_n - k;
        [candidate_nodes, removed_now, R_now] = removeRedundantNodesByRoI( ...
            candidate_nodes, selected_node, sensor_dofs, phi_s, w, R_th, minRemainingCandidates);
        removed_nodes{k+1} = removed_now; 
        R_ij_rowsum{k} = R_now; 
    end

    EfI3p_norm = zeros(length(candidate_nodes), numel(phi_s));

    for s = 1:numel(phi_s)
        h = waitbar(0, sprintf('Final placement (scenario %d/%d)...', s, numel(phi_s)));

        sel_idx = find(ismember(sensor_dofs(:,1), selected_node));
        Q0 = phi_s{s}(sel_idx,:)' * phi_s{s}(sel_idx,:);

        EfI3p = zeros(length(candidate_nodes),1);

        for i = 1:length(candidate_nodes)
            node_i_index = find(ismember(sensor_dofs(:,1), candidate_nodes(i)));
            phi_3i = phi_s{s}(node_i_index,:);

            % EFI3+ metric (FSSP)
            % Use matrix division instead of inv(Q0) for numerical stability.
            EfI3p(i) = det(eye(3) + phi_3i * (Q0 \ phi_3i')) - 1;

            waitbar(i/length(candidate_nodes), h);
        end

        close(h);

        % Normalize EFI3+ values
        EfI3p_norm(:,s) = normalizeMinMax(EfI3p);
    end

    % Scenario-weighted interpolation
    EfI3p_interp = EfI3p_norm * w(:);

    % Select next node
    [~, max_index] = max(EfI3p_interp);
    selected_node = [selected_node candidate_nodes(max_index)]; %#ok<AGROW>
    k = k + 1;

    candidate_nodes(max_index) = [];
end

runtime_Final = toc;

%% -------------------------
% Save Results
% -------------------------
resultFile = sprintf('runtime_5f_SPI_%d%d%d_%dsensors.mat', ...
    round(w_input(1)), round(w_input(2)), round(w_input(3)), sensor_n);

save(resultFile, ...
    'runtime_1st', 'runtime_Initial', 'runtime_Final', ...
    'selected_node', 'sensor_n', 'w', 'targetModes', 'rank_data', ...
    'applyRoI', 'R_th', 'removed_nodes', 'R_ij_rowsum');

disp('SPI sensor placement completed successfully.');
disp(['Selected sensor nodes: ', mat2str(selected_node)]);
disp(['Result file: ', resultFile]);

%% =========================================================
% Local helper functions
% =========================================================
function x_norm = normalizeMinMax(x)
    x_min = min(x);
    x_max = max(x);
    x_norm = (x - x_min) ./ (x_max - x_min + eps);
end


function [candidate_nodes_new, removed_nodes, R_weighted] = removeRedundantNodesByRoI( ...
    candidate_nodes, selected_node, sensor_dofs, phi_s, w, R_th, minRemainingCandidates)
% removeRedundantNodesByRoI removes candidate nodes that are redundant with
% already selected nodes based on the RoI criterion used in the original code.
%
% R_ij = rho(Q_3i - Q_3j) / rho(Q_3i + Q_3j)
%
% where rho(.) is the spectral radius. A small R_ij means that candidate node
% j has similar modal information to selected node i. Candidate nodes with
% scenario-weighted R_ij <= R_th are removed.

    candidate_nodes_new = candidate_nodes;
    removed_nodes = [];
    R_weighted = [];

    if isempty(candidate_nodes) || isempty(selected_node)
        return;
    end

    nSelected = numel(selected_node);
    nCandidate = numel(candidate_nodes);
    nScenario = numel(phi_s);

    R_ij = zeros(nSelected, nCandidate, nScenario);

    for s = 1:nScenario
        for i = 1:nSelected
            node_i_index = find(ismember(sensor_dofs(:,1), selected_node(i)));
            phi_3i = phi_s{s}(node_i_index,:);
            Q_3i = phi_3i' * phi_3i;

            for j = 1:nCandidate
                node_j_index = find(ismember(sensor_dofs(:,1), candidate_nodes(j)));
                phi_3j = phi_s{s}(node_j_index,:);
                Q_3j = phi_3j' * phi_3j;

                numerator = max(abs(eig(Q_3i - Q_3j)));
                denominator = max(abs(eig(Q_3i + Q_3j)));
                R_ij(i,j,s) = numerator / (denominator + eps);
            end
        end
    end

    % Scenario-weighted RoI matrix: nSelected x nCandidate
    R_weighted = zeros(nSelected, nCandidate);
    for s = 1:nScenario
        R_weighted = R_weighted + w(s) * R_ij(:,:,s);
    end

    % Remove a candidate if it is redundant with any selected node.
    redundantFlag = any(R_weighted <= R_th, 1);
    redundantIdx = find(redundantFlag);

    if isempty(redundantIdx)
        return;
    end

    % Safety guard: keep enough candidates to finish the requested number of sensors.
    maxRemovable = nCandidate - minRemainingCandidates;
    if maxRemovable <= 0
        return;
    end

    if numel(redundantIdx) > maxRemovable
        % Prioritize removal of the most redundant nodes, i.e., smallest min R.
        minR = min(R_weighted(:, redundantIdx), [], 1);
        [~, order] = sort(minR, 'ascend');
        redundantIdx = redundantIdx(order(1:maxRemovable));
    end

    removed_nodes = candidate_nodes(redundantIdx);
    candidate_nodes_new(redundantIdx) = [];
end

function value = getFirstAvailableVariable(dataStruct, candidateNames, sourceFile)
    for ii = 1:numel(candidateNames)
        name = candidateNames{ii};
        if isfield(dataStruct, name)
            value = dataStruct.(name);
            return;
        end
    end

    availableNames = strjoin(fieldnames(dataStruct), ', ');
    error('No valid mode-shape variable found in %s. Expected one of: %s. Available variables: %s', ...
        sourceFile, strjoin(candidateNames, ', '), availableNames);
end
