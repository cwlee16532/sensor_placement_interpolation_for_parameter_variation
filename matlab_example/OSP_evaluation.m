%% Sensor Placement Performance Evaluation for 5-Story SPI/EFI Results
% ------------------------------------------------------------
% This script evaluates one or more sensor placement result files for the
% 5-story frame structure using Fisher-information- and MAC-based metrics.
%
% Main procedure:
%   1) Define multiple sensor placement result files
%   2) Load DOF information and candidate sensor nodes
%   3) Load scenario-dependent mode-shape matrices
%   4) Evaluate performance for different numbers of sensors
%   5) Evaluate performance for all target-mode scenarios
%   6) Plot Det(FIM), condition number, and AutoMAC off-diagonal metrics
%
% Required input files in the same folder:
%   - dofs_5f.mat
%   - sensor_candidate_5f.mat
%   - system5f_100.mat
%   - system5f_80.mat
%   - system5f_60.mat
%   - system5f_40.mat
%   - system5f_20.mat
%   - one or more sensor placement result files listed in placementCases
%
% Author: Chanwoo Lee
% Repository: https://github.com/cwlee16532/sensor_placement_interpolation_for_parameter_variation
% ------------------------------------------------------------

clear; clc; close all;

%% -------------------------
% User-defined options
% -------------------------
fileDofs = 'dofs_5f.mat';
fileCandidates = 'sensor_candidate_5f.mat';

% Define sensor placement result files to compare.
% Each file should contain a vector variable such as selected_node.
% Add, remove, or edit rows depending on the available result files.
placementCases = struct( ...
    'label', { ...
        'SPI (1:1:1), 6 sensors', ...
        'SPI (1:0:0), 6 sensors', ...
        'SPI (0:1:0), 6 sensors', ...
        'SPI (0:0:1), 6 sensors', ...
        % 'SPI (1:1:1), 10 sensors' ...
    }, ...
    'file', { ...
        'runtime_5f_SPI_111_6sensors.mat', ...
        'runtime_5f_SPI_100_6sensors.mat', ...
        'runtime_5f_SPI_010_6sensors.mat', ...
        'runtime_5f_SPI_001_6sensors.mat', ...
        % 'runtime_5f_SPI_111_10sensors.mat' ...
    }, ...
    'variableName', { ...
        'selected_node', ...
        'selected_node', ...
        'selected_node', ...
        'selected_node', ... ...
        % 'selected_node', ...
        % 'selected_node', ...
        % 'selected_node' ...
    } ...
);

% Sensor numbers to evaluate.
% Values larger than the available number of sensors in a placement case are skipped.
sensorNumbers = 1:10;

% Plot settings
makePlots = true;
plotScenarioIndex = 1;   % 1: 100%, 2: 80%, 3: 60%, 4: 40%, 5: 20%
barSensorNumber = 6;     % sensor number used in scenario-wise bar plots

% Numerical settings
rankTol = 1e-12;

%% -------------------------
% Scenario-dependent mode-shape files and target modes
% -------------------------
modeFiles = { ...
    'system5f_100.mat', ...
    'system5f_80.mat',  ...
    'system5f_60.mat',  ...
    'system5f_40.mat',  ...
    'system5f_20.mat'};

modeVariableNames = {'phi', 'phi', 'phi', 'phi', 'phi'};
scenarioLabels = {'100%', '80%', '60%', '40%', '20%'};

% Target modes used for each structural scenario.
targetModes = { ...
    [1, 2, 4, 5], ...
    [1, 2, 4, 5], ...
    [1, 2, 4, 5], ...
    [1, 2, 4, 5], ...
    [1, 2, 3, 4, 5]};

%% -------------------------
% Load DOFs
% -------------------------
D = load(fileDofs, 'dofs_5f');
dofs = D.dofs_5f;

%% -------------------------
% Load sensor placement results
% -------------------------
numPlacements = numel(placementCases);
OSP = cell(numPlacements, 1);
placementLabels = cell(numPlacements, 1);

for p = 1:numPlacements
    if ~isfile(placementCases(p).file)
        error('Placement file not found: %s', placementCases(p).file);
    end

    data = load(placementCases(p).file, placementCases(p).variableName);

    if ~isfield(data, placementCases(p).variableName)
        error('Variable "%s" was not found in %s.', ...
            placementCases(p).variableName, placementCases(p).file);
    end

    OSP{p} = data.(placementCases(p).variableName)(:).';
    placementLabels{p} = placementCases(p).label;
end

% Keep only sensor numbers that are meaningful for at least one placement case.
maxAvailableSensors = max(cellfun(@numel, OSP));
sensorNumbers = sensorNumbers(sensorNumbers <= maxAvailableSensors);
barSensorNumber = min(barSensorNumber, maxAvailableSensors);

%% -------------------------
% Load candidate sensor nodes and construct candidate DOFs
% -------------------------
C = load(fileCandidates, 'sensor_candidate_5f');
candidateNodeList = C.sensor_candidate_5f;

% Extract DOFs corresponding to candidate sensor nodes.
sensor_index = find(ismember(dofs(:,1), candidateNodeList));
sensor_dofs = dofs(sensor_index,:);

% Remove rotational DOFs and retain translational DOFs only.
rot_dofs_index = find(ismember(sensor_dofs(:,2), [4, 5, 6]));
sensor_dofs(rot_dofs_index,:) = [];
sensor_node = unique(sensor_dofs(:,1));

%% -------------------------
% Load scenario-dependent mode shapes
% -------------------------
numScenarios = numel(modeFiles);
phi_all = cell(1, numScenarios);

for i = 1:numScenarios
    modeData = load(modeFiles{i}, modeVariableNames{i});
    phi_all{i} = modeData.(modeVariableNames{i});
end

%% =========================================================
% Sensor placement performance evaluation
% =========================================================
maxSensorNumber = max(sensorNumbers);

trace_Q = nan(numPlacements, maxSensorNumber, numScenarios);
det_Q = nan(numPlacements, maxSensorNumber, numScenarios);
log_det_Q = nan(numPlacements, maxSensorNumber, numScenarios);
CN_Q = nan(numPlacements, maxSensorNumber, numScenarios);
rank_Q = nan(numPlacements, maxSensorNumber, numScenarios);
max_MAC_off_diag = nan(numPlacements, maxSensorNumber, numScenarios);
mean_MAC_off_diag = nan(numPlacements, maxSensorNumber, numScenarios);
auto_MAC = cell(numPlacements, maxSensorNumber, numScenarios);

for p = 1:numPlacements
    for n = sensorNumbers
        if n > numel(OSP{p})
            continue;
        end

        selectedNodes = OSP{p}(1:n);

        for i = 1:numScenarios
            phi_full = phi_all{i};
            target_mode = targetModes{i};

            % Candidate sensor mode shapes for the current scenario.
            phi_target = phi_full(:, target_mode);
            phi_s = phi_target(sensor_index, :);
            phi_s(rot_dofs_index,:) = [];

            % Selected sensor information matrix.
            selected_node_index = find(ismember(sensor_dofs(:,1), selectedNodes));
            phi_selected = phi_s(selected_node_index,:);
            Q_selected = phi_selected' * phi_selected;

            % Fisher-information-based metrics.
            trace_Q(p,n,i) = trace(Q_selected);
            det_Q(p,n,i) = det(Q_selected);
            log_det_Q(p,n,i) = safeLogDet(Q_selected);
            CN_Q(p,n,i) = cond(Q_selected);
            rank_Q(p,n,i) = rank(Q_selected, rankTol);

            % AutoMAC-based metrics.
            auto_MAC{p,n,i} = computeMAC(phi_selected, phi_selected);
            offDiagMAC = auto_MAC{p,n,i} - eye(numel(target_mode));
            max_MAC_off_diag(p,n,i) = max(offDiagMAC, [], 'all');
            mean_MAC_off_diag(p,n,i) = mean(offDiagMAC, 'all');
        end
    end
end

%% -------------------------
% Display selected nodes and summary metrics
% -------------------------
fprintf('Sensor placement performance evaluation completed.\n');

for p = 1:numPlacements
    fprintf('\nPlacement case %d: %s\n', p, placementLabels{p});
    fprintf('File: %s\n', placementCases(p).file);
    fprintf('Selected nodes: %s\n', mat2str(OSP{p}));
end

summaryTable = buildSummaryTable(placementLabels, scenarioLabels, ...
    det_Q, log_det_Q, CN_Q, max_MAC_off_diag, rank_Q, barSensorNumber);

disp(summaryTable);

%% -------------------------
% Save processed performance metrics
% -------------------------
resultFile = 'performance_5f_multiple_sensor_placements.mat';

save(resultFile, ...
    'placementCases', 'placementLabels', 'OSP', ...
    'modeFiles', 'scenarioLabels', 'targetModes', 'sensorNumbers', ...
    'trace_Q', 'det_Q', 'log_det_Q', 'CN_Q', 'rank_Q', ...
    'auto_MAC', 'max_MAC_off_diag', 'mean_MAC_off_diag', ...
    'summaryTable', 'barSensorNumber');

fprintf('\nResult file: %s\n', resultFile);

%% -------------------------
% Plot performance versus sensor number
% -------------------------
if makePlots
    plotMetricVsSensorNumber(sensorNumbers, det_Q(:,:,plotScenarioIndex), ...
        placementLabels, 'Number of sensors', 'Det(FIM)', ...
        sprintf('Det(FIM), system %s', scenarioLabels{plotScenarioIndex}));

    plotMetricVsSensorNumber(sensorNumbers, max_MAC_off_diag(:,:,plotScenarioIndex), ...
        placementLabels, 'Number of sensors', 'Max AutoMAC off-diagonal', ...
        sprintf('Max AutoMAC off-diagonal, system %s', scenarioLabels{plotScenarioIndex}));

    plotMetricVsSensorNumber(sensorNumbers, CN_Q(:,:,plotScenarioIndex), ...
        placementLabels, 'Number of sensors', 'Condition number', ...
        sprintf('Condition number, system %s', scenarioLabels{plotScenarioIndex}));
end

%% -------------------------
% Plot scenario-wise bar charts for the selected sensor number
% -------------------------
if makePlots
    % Det(FIM) can have very different scales across structural scenarios.
    % Therefore, each scenario is plotted in a separate subplot with its own
    % y-axis scale, following the style of the original research script.
    plotScenarioBarSubplots(det_Q(:,barSensorNumber,:), placementLabels, scenarioLabels, ...
        sprintf('Det(FIM), %d sensors', barSensorNumber), 'Det(FIM)');

    % MAC and condition number are kept as grouped scenario-wise bar charts.
    plotScenarioBar(max_MAC_off_diag(:,barSensorNumber,:), placementLabels, scenarioLabels, ...
        sprintf('Max AutoMAC off-diagonal, %d sensors', barSensorNumber), ...
        'Max AutoMAC off-diagonal');

    plotScenarioBar(CN_Q(:,barSensorNumber,:), placementLabels, scenarioLabels, ...
        sprintf('Condition number, %d sensors', barSensorNumber), 'Condition number');
end

%% =========================================================
% Local helper functions
% =========================================================
function mac = computeMAC(phiA, phiB)
% computeMAC calculates the modal assurance criterion matrix.
% Columns are interpreted as mode-shape vectors.
    numerator = abs(phiA' * phiB).^2;
    denominator = diag(phiA' * phiA) * diag(phiB' * phiB).';
    mac = numerator ./ (denominator + eps);
end

function value = safeLogDet(Q)
% safeLogDet computes log(det(Q)) robustly for diagnostic use.
% If Q is not positive definite, it falls back to log(abs(det(Q))).
    Qsym = (Q + Q') / 2;
    [R, flag] = chol(Qsym);
    if flag == 0
        value = 2 * sum(log(abs(diag(R)) + eps));
    else
        value = log(abs(det(Q)) + eps);
    end
end

function plotMetricVsSensorNumber(sensorNumbers, metricMatrix, labels, xLabelText, yLabelText, titleText)
% Plot a performance metric versus the number of sensors.
    figure;
    hold on;
    for p = 1:size(metricMatrix, 1)
        y = metricMatrix(p, sensorNumbers);
        plot(sensorNumbers, y, 'o-', 'LineWidth', 1);
    end
    grid on;
    box on;
    xticks(sensorNumbers);
    xlabel(xLabelText);
    ylabel(yLabelText);
    title(titleText, 'FontSize', 10);
    legend(labels, 'Location', 'best', 'Interpreter', 'none');
end


function plotScenarioBarSubplots(metricArray, labels, scenarioLabels, titleText, yLabelText)
% Plot scenario-wise bar charts using independent y-axis scales.
% This is useful for Det(FIM), because its magnitude can vary significantly
% depending on the structural scenario and target-mode set.
% metricArray has size [numPlacements x 1 x numScenarios].
    y = squeeze(metricArray);
    if isvector(y)
        y = y(:);
    end

    numPlacements = size(y, 1);
    numScenarios = size(y, 2);
    barWidth = 0.8;

    figure;
    for s = 1:numScenarios
        subplot(1, numScenarios, s);
        hold on;

        for p = 1:numPlacements
            bar(p, y(p,s), ...
                'BarWidth', barWidth, ...
                'LineWidth', 1);
        end

        grid on;
        box on;
        title(sprintf('System %s', scenarioLabels{s}), 'FontSize', 10);
        set(gca, 'XTick', 1:numPlacements, 'XTickLabel', []);
        xlim([0.5, numPlacements + 0.5]);

        if s == 1
            ylabel(yLabelText);
        end
    end

    sgtitle(titleText, 'FontSize', 11);
    legend(labels, 'Location', 'bestoutside', 'Interpreter', 'none');
end

function plotScenarioBar(metricArray, labels, scenarioLabels, titleText, yLabelText)
% Plot scenario-wise bar charts for all placement cases.
% metricArray has size [numPlacements x 1 x numScenarios].
    y = squeeze(metricArray);
    if isvector(y)
        y = y(:).';
    end

    figure;
    bar(y.', 'LineWidth', 1);
    grid on;
    box on;
    xticklabels(scenarioLabels);
    xlabel('Structural scenario');
    ylabel(yLabelText);
    title(titleText, 'FontSize', 10);
    legend(labels, 'Location', 'best', 'Interpreter', 'none');
end

function summaryTable = buildSummaryTable(labels, scenarioLabels, det_Q, log_det_Q, CN_Q, max_MAC_off_diag, rank_Q, sensorNumber)
% Build a compact summary table for the selected sensor number.
    rows = {};
    detVals = [];
    logDetVals = [];
    cnVals = [];
    macVals = [];
    rankVals = [];

    for p = 1:numel(labels)
        for s = 1:numel(scenarioLabels)
            rows(end+1,1) = {labels{p}}; 
            rows(end,2) = {scenarioLabels{s}}; 
            detVals(end+1,1) = det_Q(p,sensorNumber,s); 
            logDetVals(end+1,1) = log_det_Q(p,sensorNumber,s); 
            cnVals(end+1,1) = CN_Q(p,sensorNumber,s); 
            macVals(end+1,1) = max_MAC_off_diag(p,sensorNumber,s); 
            rankVals(end+1,1) = rank_Q(p,sensorNumber,s); 
        end
    end

    summaryTable = table(rows(:,1), rows(:,2), detVals, logDetVals, cnVals, macVals, rankVals, ...
        'VariableNames', {'PlacementCase', 'Scenario', 'DetFIM', 'LogDetFIM', ...
        'ConditionNumber', 'MaxAutoMACOffDiagonal', 'RankFIM'});
end
