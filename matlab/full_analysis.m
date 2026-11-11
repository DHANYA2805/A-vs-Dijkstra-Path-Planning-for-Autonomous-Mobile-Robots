%% REVIEW 3 (FINAL): Complete A* vs Dijkstra path-planning framework
% Robotics and Automation (BECE312L) - A* vs Dijkstra Path Planning
%
% Builds on review2_benchmark.m and completes the remaining work:
%   PART 1  Heuristic comparison  - Dijkstra vs A* (Euclidean, Octile, Manhattan)
%                                   on the same 45 maps as Review 2 (same seeds)
%   PART 2  Faster open list      - binary heap vs linear scan, timing comparison
%   PART 3  Toolbox validation    - our A* vs MATLAB plannerAStarGrid
%                                   (skipped automatically if Navigation Toolbox is missing)
%   PART 4  Robot motion          - path smoothing + differential-drive robot
%                                   following the path with a pure-pursuit controller
%   PART 5  Decision guideline    - scenario-based rule: when Dijkstra is fine,
%                                   when A* is needed, which heuristic to use
%
% Everything except PART 3 uses core MATLAB only (R2019b or newer).
% Outputs (CSV tables, figures, GIF, guideline text) go to ./review3_results/

clear; clc; close all;

%% 0. Configuration
cfg.mapSizes    = [20 50 100];
cfg.densities   = [0.10 0.20 0.30];
cfg.densNames   = {'Sparse', 'Medium', 'Dense'};
cfg.nTrials     = 5;              % random maps per configuration
cfg.nTimingReps = 3;              % median of repeated runs
cfg.baseSeed    = 2026;           % same seeds as Review 2 -> same maps
cfg.heuristics  = {'none', 'euclidean', 'octile', 'manhattan'};
cfg.hLabels     = {'Dijkstra', 'A* Euclidean', 'A* Octile', 'A* Manhattan'};
cfg.animate     = true;           % animate the robot in PART 4
cfg.saveGif     = true;           % save the animation as a GIF
cfg.outDir      = 'review3_results';
if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end

nS = numel(cfg.mapSizes); nD = numel(cfg.densities); nH = numel(cfg.heuristics);
BYTES_PER_NODE = 8 + 4 + 1 + 1;   % g (double) + parent (uint32) + closed + open flags

%% ===================== PART 1: Heuristic comparison =====================
fprintf('PART 1: heuristic comparison on %d maps...\n', nS * nD * cfg.nTrials);
nRows = nS * nD * cfg.nTrials * nH;
SizeIdx = zeros(nRows,1); DensIdx = zeros(nRows,1); HeurIdx = zeros(nRows,1);
MapSize = zeros(nRows,1); DensityLabel = strings(nRows,1); Trial = zeros(nRows,1);
Planner = strings(nRows,1); PathLength = zeros(nRows,1); TimeMs = zeros(nRows,1);
NodesExpanded = zeros(nRows,1); PeakOpen = zeros(nRows,1); MemoryKB = zeros(nRows,1);
SuboptPct = zeros(nRows,1);

maps = cell(nS, nD, cfg.nTrials);
r = 0;
for si = 1:nS
    n = cfg.mapSizes(si);
    for di = 1:nD
        for t = 1:cfg.nTrials
            seed = cfg.baseSeed + 1000 * si + 100 * di + t;
            [occ, start, goal] = makeSolvableMap(n, cfg.densities(di), seed);
            maps{si, di, t} = struct('occ', occ, 'start', start, 'goal', goal);
            optLen = NaN;
            for hi = 1:nH
                [out, tMs] = timedSearch(occ, start, goal, cfg.heuristics{hi}, 'heap', cfg.nTimingReps);
                if hi == 1, optLen = out.length; end      % Dijkstra = true optimum
                r = r + 1;
                SizeIdx(r) = si; DensIdx(r) = di; HeurIdx(r) = hi;
                MapSize(r) = n; DensityLabel(r) = cfg.densNames{di}; Trial(r) = t;
                Planner(r) = cfg.hLabels{hi};
                PathLength(r) = out.length; TimeMs(r) = tMs;
                NodesExpanded(r) = out.nodesExpanded; PeakOpen(r) = out.peakOpen;
                MemoryKB(r) = out.nodesGenerated * BYTES_PER_NODE / 1024;
                SuboptPct(r) = 100 * (out.length / optLen - 1);
            end
        end
    end
    fprintf('  map size %dx%d done\n', n, n);
end
HT = table(SizeIdx, DensIdx, HeurIdx, MapSize, DensityLabel, Trial, Planner, PathLength, ...
           TimeMs, NodesExpanded, PeakOpen, MemoryKB, SuboptPct);
writetable(HT(:, 4:end), fullfile(cfg.outDir, 'heuristic_raw.csv'));

% Means per configuration and planner
pick  = @(si, di, hi) HT.SizeIdx == si & HT.DensIdx == di & HT.HeurIdx == hi;
avg   = @(col, si, di, hi) mean(HT.(col)(pick(si, di, hi)));
nodes = zeros(nS, nD, nH); tms = zeros(nS, nD, nH); plen = zeros(nS, nD, nH);
subMean = zeros(nS, nD, nH); subMax = zeros(nS, nD, nH); subFrac = zeros(nS, nD, nH);
for si = 1:nS
    for di = 1:nD
        for hi = 1:nH
            nodes(si,di,hi) = avg('NodesExpanded', si, di, hi);
            tms(si,di,hi)   = avg('TimeMs', si, di, hi);
            plen(si,di,hi)  = avg('PathLength', si, di, hi);
            sp = HT.SuboptPct(pick(si, di, hi));
            subMean(si,di,hi) = mean(sp);
            subMax(si,di,hi)  = max(sp);
            subFrac(si,di,hi) = 100 * mean(sp > 1e-6);
        end
    end
end
nodeRedVsDij = 100 * (1 - nodes ./ nodes(:,:,1));

fprintf('\n=== Heuristic comparison (mean over %d maps) ===\n', cfg.nTrials);
fprintf('%-8s %-7s %-14s %-8s %-9s %-9s %-11s %-10s\n', 'Size', 'Density', 'Planner', ...
        'Nodes', 'Reduc%', 'Time ms', 'PathExcess%', 'MapsSubopt%');
HS = table();
for si = 1:nS
    for di = 1:nD
        for hi = 1:nH
            fprintf('%-8s %-7s %-14s %-8.0f %-9.1f %-9.2f %-11.2f %-10.0f\n', ...
                sprintf('%dx%d', cfg.mapSizes(si), cfg.mapSizes(si)), cfg.densNames{di}, ...
                cfg.hLabels{hi}, nodes(si,di,hi), nodeRedVsDij(si,di,hi), tms(si,di,hi), ...
                subMean(si,di,hi), subFrac(si,di,hi));
            HS = [HS; table(cfg.mapSizes(si), string(cfg.densNames{di}), string(cfg.hLabels{hi}), ...
                nodes(si,di,hi), nodeRedVsDij(si,di,hi), tms(si,di,hi), plen(si,di,hi), ...
                subMean(si,di,hi), subMax(si,di,hi), subFrac(si,di,hi), ...
                'VariableNames', {'MapSize', 'Density', 'Planner', 'Nodes', 'NodeReductionPct', ...
                'TimeMs', 'PathLength', 'MeanPathExcessPct', 'MaxPathExcessPct', 'PctMapsSuboptimal'})]; %#ok<AGROW>
        end
    end
    fprintf('\n');
end
writetable(HS, fullfile(cfg.outDir, 'heuristic_summary.csv'));

% Admissibility check: Euclidean and Octile must never be suboptimal
admissOK = all(HT.SuboptPct(HT.HeurIdx == 2 | HT.HeurIdx == 3) < 1e-6);
fprintf('Admissible heuristics (Euclidean, Octile) optimal on all maps: %s\n', mat2str(admissOK));
fprintf('Manhattan: suboptimal on %d of %d maps, worst path excess %.2f%%\n\n', ...
    nnz(HT.HeurIdx == 4 & HT.SuboptPct > 1e-6), nS * nD * cfg.nTrials, ...
    max(HT.SuboptPct(HT.HeurIdx == 4)));

% Figure 1: node reduction by heuristic
fig1 = figure('Name', 'Heuristics', 'Position', [60 60 1300 450]);
hCols = [0.12 0.45 0.70; 0.11 0.62 0.47; 0.85 0.40 0.15];
for di = 1:nD
    subplot(1, nD, di);
    b = bar(squeeze(nodeRedVsDij(:, di, 2:4)), 'grouped');
    for k = 1:3, b(k).FaceColor = hCols(k, :); end
    set(gca, 'XTickLabel', compose('%dx%d', cfg.mapSizes', cfg.mapSizes'));
    ylim([0 100]); grid on;
    xlabel('Map size'); ylabel('% fewer nodes than Dijkstra');
    title(sprintf('%s obstacles (%d%%)', cfg.densNames{di}, round(100 * cfg.densities(di))));
    if di == 1, legend(cfg.hLabels(2:4), 'Location', 'northeast'); end
end
sgtitle('Search effort saved by each A* heuristic');
saveFig(fig1, fullfile(cfg.outDir, 'fig1_heuristic_nodes.png'));

% Figure 2: price of the inadmissible Manhattan heuristic
fig2 = figure('Name', 'Manhattan', 'Position', [80 80 1100 420]);
subplot(1, 2, 1);
bar(subMean(:, :, 4)); grid on;
set(gca, 'XTickLabel', compose('%dx%d', cfg.mapSizes', cfg.mapSizes'));
xlabel('Map size'); ylabel('Mean path excess over optimum (%)');
title('A* Manhattan: path length penalty'); legend(cfg.densNames, 'Location', 'northwest');
subplot(1, 2, 2);
bar(subFrac(:, :, 4)); grid on; ylim([0 100]);
set(gca, 'XTickLabel', compose('%dx%d', cfg.mapSizes', cfg.mapSizes'));
xlabel('Map size'); ylabel('% of maps with non-optimal path');
title('A* Manhattan: how often it is not optimal');
sgtitle('Manhattan distance is not admissible on an 8-connected grid');
saveFig(fig2, fullfile(cfg.outDir, 'fig2_manhattan_penalty.png'));

%% ===================== PART 2: Binary heap vs linear scan =====================
fprintf('PART 2: open-list comparison on 100x100 maps...\n');
engines = {'linear', 'heap'};
ol = zeros(nD, 2, 2);          % density x planner (Dij, A*) x engine
olMatch = true;
for di = 1:nD
    tAcc = zeros(cfg.nTrials, 2, 2);
    for t = 1:cfg.nTrials
        M = maps{nS, di, t};
        for a = 1:2
            hType = cfg.heuristics{a};          % 'none' or 'euclidean'
            lens = zeros(1, 2);
            for e = 1:2
                [out, tMs] = timedSearch(M.occ, M.start, M.goal, hType, engines{e}, cfg.nTimingReps);
                tAcc(t, a, e) = tMs;
                lens(e) = out.length;
            end
            olMatch = olMatch && abs(lens(1) - lens(2)) < 1e-6;
        end
    end
    ol(di, :, :) = mean(tAcc, 1);
end
OL = table(string(cfg.densNames'), ol(:,1,1), ol(:,1,2), ol(:,1,1) ./ ol(:,1,2), ...
           ol(:,2,1), ol(:,2,2), ol(:,2,1) ./ ol(:,2,2), ...
    'VariableNames', {'Density', 'Dij_LinearMs', 'Dij_HeapMs', 'Dij_Gain', ...
                      'AStar_LinearMs', 'AStar_HeapMs', 'AStar_Gain'});
disp(OL);
fprintf('Linear and heap versions give identical path costs: %s\n\n', mat2str(olMatch));
writetable(OL, fullfile(cfg.outDir, 'openlist_comparison.csv'));

fig3 = figure('Name', 'Open list', 'Position', [100 100 800 420]);
bar([ol(:,1,1) ol(:,1,2) ol(:,2,1) ol(:,2,2)]); grid on;
set(gca, 'XTickLabel', cfg.densNames, 'YScale', 'log');
legend({'Dijkstra - linear', 'Dijkstra - heap', 'A* - linear', 'A* - heap'}, 'Location', 'northeastoutside');
xlabel('Obstacle density (100x100 maps)'); ylabel('Planning time (ms, log scale)');
title('Binary-heap open list vs linear scan');
saveFig(fig3, fullfile(cfg.outDir, 'fig3_openlist_timing.png'));

%% ===================== PART 3: Toolbox validation =====================
fprintf('PART 3: validation against plannerAStarGrid...\n');
tbxAvailable = false;
try
    tbxAvailable = license('test', 'Navigation_Toolbox') && ~isempty(which('plannerAStarGrid'));
catch
end
if tbxAvailable
    VSize = []; VDens = strings(0,1); VTbx = []; VOurs = []; VOursCut = []; VMatch = false(0,1);
    for si = 1:nS
        for di = 1:nD
            M = maps{si, di, 1};
            try
                mapObj  = binaryOccupancyMap(M.occ);          % matrix row/col = grid row/col
                planner = plannerAStarGrid(mapObj);
                tbxPath = plan(planner, [M.start(2) M.start(1)], [M.goal(2) M.goal(1)]);
                tbxCost = sum(hypot(diff(tbxPath(:,1)), diff(tbxPath(:,2))));
                ours    = gridSearch(M.occ, M.start, M.goal, 'euclidean', 'heap', false);
                oursCut = gridSearch(M.occ, M.start, M.goal, 'euclidean', 'heap', true);
                VSize(end+1,1) = cfg.mapSizes(si);                   %#ok<SAGROW>
                VDens(end+1,1) = cfg.densNames{di};                  %#ok<SAGROW>
                VTbx(end+1,1) = tbxCost; VOurs(end+1,1) = ours.length; VOursCut(end+1,1) = oursCut.length; %#ok<SAGROW>
                VMatch(end+1,1) = abs(tbxCost - ours.length) < 1e-6 || abs(tbxCost - oursCut.length) < 1e-6; %#ok<SAGROW>
            catch ME
                fprintf('  validation failed on %dx%d %s: %s\n', cfg.mapSizes(si), ...
                        cfg.mapSizes(si), cfg.densNames{di}, ME.message);
            end
        end
    end
    VT = table(VSize, VDens, VTbx, VOurs, VOursCut, VMatch, 'VariableNames', ...
        {'MapSize', 'Density', 'ToolboxCost', 'OurCost_NoCornerCut', 'OurCost_CornerCut', 'Match'});
    disp(VT);
    fprintf('Toolbox cost matched on %d of %d maps.\n', nnz(VMatch), numel(VMatch));
    fprintf('(If the toolbox matches the corner-cut column, it allows diagonal moves\n');
    fprintf(' between two obstacles; our planner forbids this for physical safety.)\n\n');
    writetable(VT, fullfile(cfg.outDir, 'toolbox_validation.csv'));
else
    fprintf('  Navigation Toolbox not available - validation skipped.\n\n');
end

%% ===================== PART 4: Path smoothing + robot motion =====================
fprintf('PART 4: robot motion simulation...\n');
M = maps{2, 2, 1};                                  % 50x50, medium density
plan4 = gridSearch(M.occ, M.start, M.goal, 'octile', 'heap', false);
rawPath    = plan4.path;
smoothPth  = smoothPath(rawPath, M.occ, 0.35);      % 0.35 cell clearance on both sides

robot.v       = 1.0;     % cruise speed (cells/s)
robot.L       = 1.5;     % pure-pursuit lookahead (cells)
robot.wMax    = 2.5;     % max turn rate (rad/s)
robot.b       = 0.5;     % wheel base (cells)
robot.dt      = 0.05;    % time step (s)
robot.goalTol = 0.3;     % goal tolerance (cells)
robot.maxTime = 3 * plan4.length / robot.v + 20;

simRaw    = simulateRobot(rawPath,   M.occ, robot);
simSmooth = simulateRobot(smoothPth, M.occ, robot);

labels = {'Raw A* path', 'Smoothed path'};
sims = {simRaw, simSmooth}; paths = {rawPath, smoothPth};
MT = table();
for k = 1:2
    s = sims{k}; p = paths{k};
    MT = [MT; table(string(labels{k}), size(p, 1), pathLen(p), rad2deg(pathTurning(p)), ...
        s.time, s.distance, s.maxCTE, s.meanCTE, s.minClear, s.reached, s.collided, ...
        'VariableNames', {'Path', 'Waypoints', 'PlannedLength', 'TotalTurningDeg', ...
        'TravelTimeS', 'DistanceTravelled', 'MaxTrackingError', 'MeanTrackingError', ...
        'MinClearance', 'ReachedGoal', 'Collided'})]; %#ok<AGROW>
end
disp(MT);
writetable(MT, fullfile(cfg.outDir, 'motion_metrics.csv'));

fig4 = figure('Name', 'Robot motion', 'Position', [60 60 1300 600]);
subplot(2, 2, [1 3]);
plotMap(M.occ); hold on;
h1 = plot(rawPath(:,1), rawPath(:,2), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
h2 = plot(smoothPth(:,1), smoothPth(:,2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 4);
h3 = plot(simSmooth.traj(:,1), simSmooth.traj(:,2), 'r-', 'LineWidth', 2);
plot(M.start(1), M.start(2), 'bo', 'MarkerFaceColor', 'b');
plot(M.goal(1), M.goal(2), 'ro', 'MarkerFaceColor', 'r');
legend([h1 h2 h3], {'Raw A* path', 'Smoothed path', 'Robot trajectory'}, 'Location', 'southeast');
title(sprintf('Robot following smoothed A* path (50x50, medium) - %d to %d waypoints', ...
    size(rawPath, 1), size(smoothPth, 1)));
subplot(2, 2, 2);
tt = simSmooth.t;
plot(tt, simSmooth.vR, 'LineWidth', 1.5); hold on;
plot(tt, simSmooth.vL, 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Wheel speed (cells/s)');
legend('Right wheel', 'Left wheel'); title('Wheel speeds (differential drive)');
subplot(2, 2, 4);
plot(simRaw.t, simRaw.cte, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); hold on;
plot(simSmooth.t, simSmooth.cte, 'r', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Tracking error (cells)');
legend('Raw path', 'Smoothed path'); title('Distance from planned path');
saveFig(fig4, fullfile(cfg.outDir, 'fig4_robot_motion.png'));

if cfg.animate
    gifFile = '';
    if cfg.saveGif, gifFile = fullfile(cfg.outDir, 'robot_animation.gif'); end
    animateRobot(M.occ, smoothPth, simSmooth, M.start, M.goal, gifFile);
end

%% ===================== PART 5: Decision guideline =====================
fprintf('\nPART 5: decision guideline\n');
% Best admissible heuristic overall (fewest mean nodes)
admissible = [2 3];
[~, bi] = min([mean(nodes(:,:,2), 'all'), mean(nodes(:,:,3), 'all')]);
bestH = admissible(bi);

speedBest = tms(:,:,1) ./ tms(:,:,bestH);
redBest   = nodeRedVsDij(:,:,bestH);
rec = zeros(nS, nD);   % 1 = either is fine, 2 = A* recommended, 3 = A* strongly recommended
catNames = {'Either is fine', 'A* recommended', 'A* strongly recommended'};
for si = 1:nS
    for di = 1:nD
        if tms(si, di, 1) < 1                 % Dijkstra already under 1 ms
            rec(si, di) = 1;
        elseif speedBest(si, di) >= 2
            rec(si, di) = 3;
        else
            rec(si, di) = 2;
        end
    end
end

GL = table();
for si = 1:nS
    for di = 1:nD
        GL = [GL; table(cfg.mapSizes(si), string(cfg.densNames{di}), tms(si,di,1), ...
            tms(si,di,bestH), redBest(si,di), speedBest(si,di), string(catNames{rec(si,di)}), ...
            'VariableNames', {'MapSize', 'Density', 'DijkstraMs', 'AStarMs', ...
            'NodeReductionPct', 'Speedup', 'Recommendation'})]; %#ok<AGROW>
    end
end
disp(GL);
writetable(GL, fullfile(cfg.outDir, 'decision_guideline.csv'));

fig5 = figure('Name', 'Guideline', 'Position', [120 120 760 520]);
imagesc(rec, [1 3]);
colormap(gca, [0.80 0.90 0.95; 0.45 0.72 0.85; 0.13 0.35 0.55]);
set(gca, 'XTick', 1:nD, 'XTickLabel', cfg.densNames, 'YTick', 1:nS, ...
    'YTickLabel', compose('%dx%d', cfg.mapSizes', cfg.mapSizes'));
xlabel('Obstacle density'); ylabel('Map size');
for si = 1:nS
    for di = 1:nD
        txtCol = 'k'; if rec(si, di) == 3, txtCol = 'w'; end
        text(di, si, sprintf('%s\n%.1fx faster, %.0f%% fewer nodes', catNames{rec(si,di)}, ...
            speedBest(si,di), redBest(si,di)), 'HorizontalAlignment', 'center', ...
            'FontWeight', 'bold', 'Color', txtCol, 'FontSize', 9);
    end
end
title(sprintf('Decision guideline (A* uses %s heuristic)', cfg.hLabels{bestH}));
saveFig(fig5, fullfile(cfg.outDir, 'fig5_decision_guideline.png'));

% Written guideline
fid = fopen(fullfile(cfg.outDir, 'guideline.txt'), 'w');
lines = {
    'DECISION GUIDELINE - A* vs Dijkstra for grid-based mobile robot planning'
    ''
    sprintf('1. Heuristic: use %s. It is admissible on an 8-connected grid, so paths stay optimal,', cfg.hLabels{bestH})
    sprintf('   and it expanded the fewest nodes of the admissible heuristics (mean %.0f vs %.0f).', ...
        mean(nodes(:,:,bestH), 'all'), mean(nodes(:,:,5 - bestH), 'all'))
    sprintf('2. Avoid Manhattan when diagonal moves are allowed: it was non-optimal on %.0f%% of maps', ...
        mean(subFrac(:,:,4), 'all'))
    sprintf('   (worst case %.2f%% longer path), although it expands fewer nodes.', max(subMax(:,:,4), [], 'all'))
    sprintf('3. A* gives the largest benefit on large, open maps (up to %.1fx faster, %.0f%% fewer nodes).', ...
        max(speedBest, [], 'all'), max(redBest, [], 'all'))
    sprintf('4. On dense maps the benefit drops (%.0f-%.0f%% fewer nodes) because obstacles make', ...
        min(redBest(:, nD)), max(redBest(:, nD)))
    '   the straight-line estimate less accurate.'
    '5. On small maps where Dijkstra already plans in under 1 ms, either algorithm is fine.'
    '6. Dijkstra remains the better choice when costs to MANY goals are needed at once'
    '   (e.g. choosing the nearest of several charging docks), since one run gives distances to all cells.'
    '7. Use a binary-heap open list for any real deployment: it gives the same paths and is much faster.'
    };
fprintf(fid, '%s\n', lines{:});
fclose(fid);
fprintf('%s\n', lines{:});
fprintf('\nAll results saved to ./%s/\n', cfg.outDir);

%% ============================ Local functions ============================

function out = gridSearch(occ, start, goal, hType, engine, cornerCut)
% Best-first search on an 8-connected occupancy grid (occ(y,x) = true -> obstacle).
%   hType     : 'none' (Dijkstra) | 'euclidean' | 'octile' | 'manhattan' (A*)
%   engine    : 'heap' (binary heap, lazy deletion) | 'linear' (Review 2 linear scan)
%   cornerCut : false -> diagonal move needs both side cells free (default)
if nargin < 5, engine = 'heap'; end
if nargin < 6, cornerCut = false; end
useHeap = strcmpi(engine, 'heap');
[rows, cols] = size(occ);
N = rows * cols;
H = heuristicGrid(rows, cols, goal, hType);

g      = inf(N, 1);
parent = zeros(N, 1, 'uint32');
closed = false(N, 1);
inOpen = false(N, 1);
sIdx = sub2ind([rows cols], start(2), start(1));
gIdx = sub2ind([rows cols], goal(2),  goal(1));
g(sIdx) = 0;

if useHeap
    cap = 8 * N + 1;                 % each relaxation pushes once -> at most 8N entries
    hKey = zeros(cap, 1); hNode = zeros(cap, 1);
    hN = 1; hKey(1) = H(sIdx); hNode(1) = sIdx;
else
    fOpen = inf(N, 1);
    fOpen(sIdx) = H(sIdx);
end
inOpen(sIdx) = true;
openCount = 1; peakOpen = 1; nodesExpanded = 0; found = false;
moves = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; ...
         1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

while true
    % ---------- pop the node with the lowest f ----------
    if useHeap
        cur = 0;
        while hN > 0
            cand = hNode(1);
            hKey(1) = hKey(hN); hNode(1) = hNode(hN); hN = hN - 1;
            i = 1;
            while true                           % sift down
                l = 2 * i;
                if l > hN, break; end
                c = l;
                if l + 1 <= hN && hKey(l + 1) < hKey(l), c = l + 1; end
                if hKey(i) <= hKey(c), break; end
                tk = hKey(i); hKey(i) = hKey(c); hKey(c) = tk;
                tn = hNode(i); hNode(i) = hNode(c); hNode(c) = tn;
                i = c;
            end
            if ~closed(cand), cur = cand; break; end   % skip outdated duplicates
        end
        if cur == 0, break; end
    else
        [fMin, cur] = min(fOpen);
        if isinf(fMin), break; end
        fOpen(cur) = inf;
    end
    inOpen(cur) = false; openCount = openCount - 1;
    closed(cur) = true; nodesExpanded = nodesExpanded + 1;
    if cur == gIdx, found = true; break; end

    % ---------- expand neighbours ----------
    cy = mod(cur - 1, rows) + 1;
    cx = floor((cur - 1) / rows) + 1;
    for m = 1:8
        nx = cx + moves(m, 1);
        ny = cy + moves(m, 2);
        if nx < 1 || nx > cols || ny < 1 || ny > rows, continue; end
        nIdx = ny + (nx - 1) * rows;
        if occ(nIdx) || closed(nIdx), continue; end
        if ~cornerCut && moves(m,1) ~= 0 && moves(m,2) ~= 0 && (occ(cy, nx) || occ(ny, cx))
            continue
        end
        gNew = g(cur) + moves(m, 3);
        if gNew < g(nIdx)
            g(nIdx) = gNew;
            parent(nIdx) = cur;
            key = gNew + H(nIdx);
            if ~inOpen(nIdx)
                inOpen(nIdx) = true; openCount = openCount + 1;
            end
            if useHeap
                hN = hN + 1; i = hN; hKey(i) = key; hNode(i) = nIdx;
                while i > 1                      % sift up
                    p = floor(i / 2);
                    if hKey(p) <= hKey(i), break; end
                    tk = hKey(i); hKey(i) = hKey(p); hKey(p) = tk;
                    tn = hNode(i); hNode(i) = hNode(p); hNode(p) = tn;
                    i = p;
                end
            else
                fOpen(nIdx) = key;
            end
        end
    end
    peakOpen = max(peakOpen, openCount);
end

out.found          = found;
out.nodesExpanded  = nodesExpanded;
out.nodesGenerated = nnz(~isinf(g));
out.peakOpen       = peakOpen;
out.explored       = reshape(closed, rows, cols);
out.path           = zeros(0, 2);
out.length         = inf;
if found
    idxList = gIdx; cur = gIdx;
    while cur ~= sIdx
        cur = double(parent(cur));
        idxList(end + 1) = cur; %#ok<AGROW>
    end
    idxList = flip(idxList(:));
    [py, px] = ind2sub([rows cols], idxList);
    out.path   = [px py];
    out.length = g(gIdx);
end
end

function H = heuristicGrid(rows, cols, goal, hType)
[X, Y] = meshgrid(1:cols, 1:rows);
dx = abs(X - goal(1)); dy = abs(Y - goal(2));
switch lower(hType)
    case 'none',      H = zeros(rows, cols);
    case 'euclidean', H = hypot(dx, dy);
    case 'octile',    H = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy);
    case 'manhattan', H = dx + dy;
    otherwise, error('Unknown heuristic "%s".', hType);
end
end

function [out, tMs] = timedSearch(occ, start, goal, hType, engine, reps)
times = zeros(reps, 1);
for k = 1:reps
    t0 = tic;
    out = gridSearch(occ, start, goal, hType, engine, false);
    times(k) = toc(t0);
end
tMs = 1000 * median(times);
end

function [occ, start, goal] = makeSolvableMap(n, density, seed)
% Identical generator to Review 2, so the same seeds give the same maps.
start = [2, 2];
goal  = [n - 1, n - 1];
for attempt = 0:50
    rng(seed + 7919 * attempt);
    occ = rand(n) < density;
    occ = clearAround(occ, start);
    occ = clearAround(occ, goal);
    probe = gridSearch(occ, start, goal, 'none', 'heap', false);
    if probe.found, return; end
end
error('Could not generate a solvable %dx%d map at density %.2f.', n, n, density);
end

function occ = clearAround(occ, p)
[rows, cols] = size(occ);
occ(max(1, p(2)-1):min(rows, p(2)+1), max(1, p(1)-1):min(cols, p(1)+1)) = false;
end

function sp = smoothPath(path, occ, clearance)
% Line-of-sight shortcutting: from each kept waypoint, jump to the farthest
% later waypoint that can be reached in a straight, collision-free line.
n = size(path, 1);
sp = path(1, :);
i = 1;
while i < n
    j = n;
    while j > i + 1 && ~lineFree(path(i, :), path(j, :), occ, clearance)
        j = j - 1;
    end
    sp(end + 1, :) = path(j, :); %#ok<AGROW>
    i = j;
end
end

function ok = lineFree(p, q, occ, clearance)
% True if the straight segment p->q, widened by +/- clearance, hits no obstacle.
[rows, cols] = size(occ);
d = q - p;
L = hypot(d(1), d(2));
if L == 0, ok = true; return; end
nSamp = max(2, ceil(L / 0.05));
s = linspace(0, 1, nSamp)';
pts = p + s .* d;
nrm = [-d(2) d(1)] / L;
ok = true;
for off = [0, clearance, -clearance]
    q2 = pts + off * nrm;
    cx = round(q2(:, 1)); cy = round(q2(:, 2));
    if any(cx < 1 | cx > cols | cy < 1 | cy > rows), ok = false; return; end
    if any(occ(sub2ind([rows cols], cy, cx))), ok = false; return; end
end
end

function sim = simulateRobot(path, occ, p)
% Differential-drive robot (unicycle model) tracking the path with pure pursuit.
ref = resamplePath(path, 0.1);
nRef = size(ref, 1);
[rows, cols] = size(occ);
[oy, ox] = find(occ);
x = ref(1, 1); y = ref(1, 2);
ahead = min(10, nRef);
th = atan2(ref(ahead, 2) - y, ref(ahead, 1) - x);
goal = ref(end, :);

nMax = ceil(p.maxTime / p.dt);
traj = zeros(nMax, 3); vw = zeros(nMax, 2); cte = zeros(nMax, 1); clr = zeros(nMax, 1);
prog = 1; reached = false; collided = false; k = 0;
for k = 1:nMax
    % closest reference point, searched forward only (no going back)
    win = prog:min(prog + 60, nRef);
    [dmin, rel] = min(hypot(ref(win, 1) - x, ref(win, 2) - y));
    prog = win(rel);
    % lookahead point
    la = prog;
    while la < nRef && hypot(ref(la, 1) - x, ref(la, 2) - y) < p.L
        la = la + 1;
    end
    alpha = wrapAngle(atan2(ref(la, 2) - y, ref(la, 1) - x) - th);
    distGoal = hypot(goal(1) - x, goal(2) - y);
    if distGoal < p.goalTol, reached = true; k = k - 1; break; end

    v = p.v * min(1, max(0.2, distGoal / p.L));     % slow down near the goal
    w = 2 * v * sin(alpha) / p.L;                   % pure-pursuit curvature law
    w = max(-p.wMax, min(p.wMax, w));

    x  = x + v * cos(th) * p.dt;                    % unicycle kinematics
    y  = y + v * sin(th) * p.dt;
    th = wrapAngle(th + w * p.dt);

    traj(k, :) = [x y th]; vw(k, :) = [v w]; cte(k) = dmin;
    if isempty(ox), clr(k) = inf; else, clr(k) = min(hypot(ox - x, oy - y)) - 0.5; end
    cxr = round(x); cyr = round(y);
    if cxr < 1 || cxr > cols || cyr < 1 || cyr > rows || occ(cyr, cxr)
        collided = true;
    end
end
k = max(k, 1);
traj = traj(1:k, :); vw = vw(1:k, :); cte = cte(1:k); clr = clr(1:k);
sim.traj = traj;
sim.t = (1:k)' * p.dt;
sim.v = vw(:, 1); sim.w = vw(:, 2);
sim.vR = vw(:, 1) + vw(:, 2) * p.b / 2;
sim.vL = vw(:, 1) - vw(:, 2) * p.b / 2;
sim.cte = cte;
sim.time = k * p.dt;
sim.distance = sum(hypot(diff(traj(:, 1)), diff(traj(:, 2))));
sim.maxCTE = max(cte); sim.meanCTE = mean(cte);
sim.minClear = min(clr);
sim.reached = reached; sim.collided = collided;
end

function rp = resamplePath(path, ds)
seg = hypot(diff(path(:, 1)), diff(path(:, 2)));
keep = [true; seg > 0];
path = path(keep, :);
s = [0; cumsum(seg(seg > 0))];
sq = (0:ds:s(end))';
if sq(end) < s(end), sq(end + 1) = s(end); end
rp = [interp1(s, path(:, 1), sq), interp1(s, path(:, 2), sq)];
end

function a = wrapAngle(a)
a = mod(a + pi, 2 * pi) - pi;
end

function L = pathLen(path)
L = sum(hypot(diff(path(:, 1)), diff(path(:, 2))));
end

function T = pathTurning(path)
% Sum of absolute heading changes at the waypoints (radians).
d = diff(path);
d = d(any(d ~= 0, 2), :);
ang = atan2(d(:, 2), d(:, 1));
T = sum(abs(wrapAngle(diff(ang))));
end

function plotMap(occ)
[rows, cols] = size(occ);
img = repmat(double(~occ), 1, 1, 3);
image([1 cols], [1 rows], img);
axis xy equal tight;
xlabel('X'); ylabel('Y');
end

function animateRobot(occ, path, sim, start, goal, gifFile)
fig = figure('Name', 'Robot animation', 'Position', [150 80 700 680]);
lightTheme(fig);
plotMap(occ); hold on;
plot(path(:, 1), path(:, 2), 'b--', 'LineWidth', 1.5);
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b');
plot(goal(1), goal(2), 'ro', 'MarkerFaceColor', 'r');
trail = plot(nan, nan, 'r-', 'LineWidth', 2);
body  = patch(nan, nan, [0.1 0.6 0.3], 'EdgeColor', 'k');
title('Differential-drive robot following the smoothed A* path');
shape = [0.8 0; -0.5 0.45; -0.5 -0.45];          % triangle pointing forward
nK = size(sim.traj, 1);
step = max(1, round(nK / 150));                   % about 150 frames
first = true;
for k = [1:step:nK, nK]
    x = sim.traj(k, 1); y = sim.traj(k, 2); th = sim.traj(k, 3);
    R = [cos(th) -sin(th); sin(th) cos(th)];
    pts = (R * shape')' + [x y];
    set(body, 'XData', pts(:, 1), 'YData', pts(:, 2));
    set(trail, 'XData', sim.traj(1:k, 1), 'YData', sim.traj(1:k, 2));
    drawnow;
    if ~isempty(gifFile)
        [A, cmap] = rgb2ind(frame2im(getframe(fig)), 256);
        if first
            imwrite(A, cmap, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.04);
            first = false;
        else
            imwrite(A, cmap, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.04);
        end
    end
end
end

function lightTheme(fig)
% Light figures for slides (theme() exists in R2025a+; ignored on older versions).
try
    theme(fig, 'light');
catch
end
end

function saveFig(fig, fileName)
lightTheme(fig);
drawnow;
saveas(fig, fileName);
end
