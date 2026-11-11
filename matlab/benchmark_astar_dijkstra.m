%% REVIEW 2: Multi-scenario, multi-metric benchmark of A* vs Dijkstra
% Robotics and Automation (BECE312L) - A* vs Dijkstra Path Planning
%
% What this script does (Review 2 scope, ~75% of the project):
%   1. Generates random occupancy grids for 3 map sizes x 3 obstacle
%      densities = 9 configurations, several random maps per configuration
%      (seeded, so every run is reproducible).
%   2. Runs Dijkstra and A* on the SAME map, start and goal.
%   3. Logs 4 metrics per run: path length, planning time, nodes expanded,
%      and memory (peak open list + nodes stored).
%   4. Checks optimality: A* (admissible heuristic) must give the same path
%      cost as Dijkstra on every map.
%   5. Saves raw + summary CSVs and figures to ./review2_results/
%
% Left for Review 3 (hooks already present): heuristic comparison
% (euclidean / octile / manhattan), robot motion simulation along the path,
% validation against the toolbox plannerAStarGrid, binary-heap open list.
%
% No toolboxes required. Tested logic targets MATLAB R2019b or newer.

clear; clc; close all;

%% 0. Configuration
cfg.mapSizes    = [20 50 100];
cfg.densities   = [0.10 0.20 0.30];
cfg.densNames   = {'Sparse', 'Medium', 'Dense'};
cfg.nTrials     = 5;          % random maps per configuration
cfg.nTimingReps = 3;          % repeat each run, keep the median time
cfg.baseSeed    = 2026;
cfg.heuristic   = 'euclidean';% A* heuristic used in Review 2
cfg.outDir      = 'review2_results';
if ~exist(cfg.outDir, 'dir'), mkdir(cfg.outDir); end

algoNames = {'Dijkstra', 'AStar'};
nS = numel(cfg.mapSizes); nD = numel(cfg.densities); nA = numel(algoNames);
nRows = nS * nD * cfg.nTrials * nA;

% Raw results (one row per algorithm per map)
MapSize        = zeros(nRows, 1);
Density        = zeros(nRows, 1);
DensityLabel   = strings(nRows, 1);
Trial          = zeros(nRows, 1);
Algorithm      = strings(nRows, 1);
PathLength     = zeros(nRows, 1);
TimeMs         = zeros(nRows, 1);
NodesExpanded  = zeros(nRows, 1);
NodesGenerated = zeros(nRows, 1);
PeakOpen       = zeros(nRows, 1);
MemoryKB       = zeros(nRows, 1);

BYTES_PER_NODE = 8 + 8 + 4 + 1;   % g (double) + f (double) + parent (uint32) + closed flag
showcase = cell(1, nS);           % one representative map per size for the overlay figure
optimalityFailures = 0;
r = 0;

fprintf('Running %d configurations x %d trials...\n', nS * nD, cfg.nTrials);
for si = 1:nS
    n = cfg.mapSizes(si);
    for di = 1:nD
        for t = 1:cfg.nTrials
            seed = cfg.baseSeed + 1000 * si + 100 * di + t;
            [occ, start, goal] = makeSolvableMap(n, cfg.densities(di), seed);

            res = cell(1, nA);
            for a = 1:nA
                if strcmp(algoNames{a}, 'Dijkstra')
                    hType = 'none';
                else
                    hType = cfg.heuristic;
                end

                times = zeros(cfg.nTimingReps, 1);
                for k = 1:cfg.nTimingReps
                    tStart = tic;
                    out = gridSearch(occ, start, goal, hType);
                    times(k) = toc(tStart);
                end

                r = r + 1;
                MapSize(r)        = n;
                Density(r)        = cfg.densities(di);
                DensityLabel(r)   = cfg.densNames{di};
                Trial(r)          = t;
                Algorithm(r)      = algoNames{a};
                PathLength(r)     = out.length;
                TimeMs(r)         = 1000 * median(times);
                NodesExpanded(r)  = out.nodesExpanded;
                NodesGenerated(r) = out.nodesGenerated;
                PeakOpen(r)       = out.peakOpen;
                MemoryKB(r)       = out.nodesGenerated * BYTES_PER_NODE / 1024;
                res{a} = out;
            end

            % Optimality check: admissible A* must match Dijkstra's cost
            if abs(res{1}.length - res{2}.length) > 1e-6
                optimalityFailures = optimalityFailures + 1;
                warning('Cost mismatch: size %d, %s, trial %d (D=%.3f, A*=%.3f)', ...
                    n, cfg.densNames{di}, t, res{1}.length, res{2}.length);
            end

            % Keep medium-density, first-trial map of each size for visuals
            if di == 2 && t == 1
                showcase{si} = struct('occ', occ, 'start', start, 'goal', goal, ...
                                      'dij', res{1}, 'ast', res{2}, 'n', n);
            end
        end
    end
    fprintf('  map size %dx%d done\n', n, n);
end

T = table(MapSize, Density, DensityLabel, Trial, Algorithm, PathLength, TimeMs, ...
          NodesExpanded, NodesGenerated, PeakOpen, MemoryKB);
writetable(T, fullfile(cfg.outDir, 'results_raw.csv'));

%% 1. Summarise: mean of each metric per configuration and algorithm
meanOf = @(col, n, dens, algo) mean(T.(col)(T.MapSize == n & ...
                     T.Density == dens & T.Algorithm == algo));
metrics = {'PathLength', 'TimeMs', 'NodesExpanded', 'PeakOpen', 'MemoryKB'};
M = struct();
for m = 1:numel(metrics)
    M.(metrics{m}) = zeros(nS, nD, nA);
    for si = 1:nS
        for di = 1:nD
            for a = 1:nA
                M.(metrics{m})(si, di, a) = meanOf(metrics{m}, cfg.mapSizes(si), ...
                                                   cfg.densities(di), algoNames{a});
            end
        end
    end
end

nodeRed = 100 * (1 - M.NodesExpanded(:,:,2) ./ M.NodesExpanded(:,:,1));  % % fewer nodes
memRed  = 100 * (1 - M.MemoryKB(:,:,2)      ./ M.MemoryKB(:,:,1));
speedup = M.TimeMs(:,:,1) ./ M.TimeMs(:,:,2);                             % x faster

fprintf('\n=== Summary (mean over %d maps per configuration) ===\n', cfg.nTrials);
fprintf('%-6s %-7s | %-9s %-9s %-8s | %-9s %-9s %-7s | %-7s | %-9s\n', ...
    'Size', 'Density', 'Dij nodes', 'A* nodes', 'Reduc%', ...
    'Dij ms', 'A* ms', 'Speedup', 'MemRed%', 'Path len');
Summary = table();
for si = 1:nS
    for di = 1:nD
        fprintf('%-6d %-7s | %-9.0f %-9.0f %-8.1f | %-9.2f %-9.2f %-7.2f | %-7.1f | %-9.2f\n', ...
            cfg.mapSizes(si), cfg.densNames{di}, ...
            M.NodesExpanded(si,di,1), M.NodesExpanded(si,di,2), nodeRed(si,di), ...
            M.TimeMs(si,di,1), M.TimeMs(si,di,2), speedup(si,di), ...
            memRed(si,di), M.PathLength(si,di,2));
        Summary = [Summary; table(cfg.mapSizes(si), string(cfg.densNames{di}), ...
            M.NodesExpanded(si,di,1), M.NodesExpanded(si,di,2), nodeRed(si,di), ...
            M.TimeMs(si,di,1), M.TimeMs(si,di,2), speedup(si,di), memRed(si,di), ...
            M.PathLength(si,di,2), 'VariableNames', {'MapSize', 'Density', ...
            'DijNodes', 'AStarNodes', 'NodeReductionPct', 'DijTimeMs', 'AStarTimeMs', ...
            'Speedup', 'MemReductionPct', 'PathLength'})]; %#ok<AGROW>
    end
end
writetable(Summary, fullfile(cfg.outDir, 'results_summary.csv'));
fprintf('\nOptimality check: %d mismatches out of %d maps (should be 0).\n', ...
    optimalityFailures, nS * nD * cfg.nTrials);

%% 2. Figure: nodes expanded and time vs map size, one panel per density
fig1 = figure('Name', 'Scaling', 'Position', [80 80 1200 650]);
for di = 1:nD
    subplot(2, nD, di);
    semilogy(cfg.mapSizes, M.NodesExpanded(:,di,1), 'm-o', 'LineWidth', 1.8); hold on;
    semilogy(cfg.mapSizes, M.NodesExpanded(:,di,2), 'g-s', 'LineWidth', 1.8);
    grid on; xticks(cfg.mapSizes);
    xlabel('Map size (N x N)'); ylabel('Nodes expanded');
    title(sprintf('%s obstacles (%d%%)', cfg.densNames{di}, round(100*cfg.densities(di))));
    legend('Dijkstra', 'A*', 'Location', 'northwest');

    subplot(2, nD, nD + di);
    semilogy(cfg.mapSizes, M.TimeMs(:,di,1), 'm-o', 'LineWidth', 1.8); hold on;
    semilogy(cfg.mapSizes, M.TimeMs(:,di,2), 'g-s', 'LineWidth', 1.8);
    grid on; xticks(cfg.mapSizes);
    xlabel('Map size (N x N)'); ylabel('Planning time (ms)');
    legend('Dijkstra', 'A*', 'Location', 'northwest');
end
sgtitle('Scaling of search effort with map size');
saveas(fig1, fullfile(cfg.outDir, 'fig1_scaling.png'));

%% 3. Figure: node-reduction heatmap across the 9 configurations
fig2 = figure('Name', 'Reduction', 'Position', [120 120 900 380]);
panels = {nodeRed, speedup};
panelTitles = {'% fewer nodes expanded by A*', 'Speedup of A* (Dijkstra time / A* time)'};
panelFmt = {'%.1f%%', '%.2fx'};
for p = 1:2
    subplot(1, 2, p);
    imagesc(panels{p}); colormap(gca, parula); colorbar;
    set(gca, 'XTick', 1:nD, 'XTickLabel', cfg.densNames, ...
             'YTick', 1:nS, 'YTickLabel', compose('%dx%d', cfg.mapSizes', cfg.mapSizes'));
    xlabel('Obstacle density'); ylabel('Map size');
    title(panelTitles{p});
    for si = 1:nS
        for di = 1:nD
            text(di, si, sprintf(panelFmt{p}, panels{p}(si,di)), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w');
        end
    end
end
saveas(fig2, fullfile(cfg.outDir, 'fig2_reduction_heatmap.png'));

%% 4. Figure: search-space overlay (the "visual proof" from Review 1)
fig3 = figure('Name', 'Search space', 'Position', [60 60 1400 480]);
for si = 1:nS
    s = showcase{si};
    subplot(1, nS, si);
    plotSearchOverlay(s.occ, s.dij, s.ast);
    hold on;
    hD = plot(s.dij.path(:,1), s.dij.path(:,2), 'm-', 'LineWidth', 2);
    hA = plot(s.ast.path(:,1), s.ast.path(:,2), 'g--', 'LineWidth', 2);
    plot(s.start(1), s.start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);
    plot(s.goal(1),  s.goal(2),  'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
    title(sprintf('%dx%d, medium density\nDijkstra: %d nodes | A*: %d nodes', ...
        s.n, s.n, s.dij.nodesExpanded, s.ast.nodesExpanded));
    if si == 1
        legend([hD hA], {'Dijkstra path', 'A* path'}, 'Location', 'southeast');
    end
end
sgtitle({'Explored cells: light blue = Dijkstra only, orange = both, yellow = A* only'});
saveas(fig3, fullfile(cfg.outDir, 'fig3_search_space_overlay.png'));

fprintf('Results and figures saved to ./%s/\n', cfg.outDir);

%% ======================= Local functions =======================

function out = gridSearch(occ, start, goal, hType)
% Best-first grid search on an 8-connected grid.
%   hType = 'none'      -> Dijkstra (uniform-cost search)
%   hType = 'euclidean' -> A* (admissible and consistent)
%   hType = 'octile'    -> A* (admissible, tightest for 8-connected moves)
%   hType = 'manhattan' -> A* (NOT admissible with diagonal moves - Review 3)
% start/goal are [x y]. Diagonal moves may not cut obstacle corners.
[rows, cols] = size(occ);
H = heuristicGrid(rows, cols, goal, hType);

g       = inf(rows, cols);            % cost from start
fOpen   = inf(rows, cols);            % f of open nodes; inf = not in open list
parent  = zeros(rows, cols, 'uint32');% linear index of predecessor
closed  = false(rows, cols);

sIdx = sub2ind([rows cols], start(2), start(1));
gIdx = sub2ind([rows cols], goal(2),  goal(1));
g(sIdx) = 0;
fOpen(sIdx) = H(sIdx);

openCount = 1; peakOpen = 1; nodesExpanded = 0; found = false;
moves = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; ...
         1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

while true
    [fMin, cur] = min(fOpen(:));      % pop lowest f (linear scan, O(N))
    if isinf(fMin), break; end        % open list empty -> no path
    fOpen(cur) = inf;
    openCount = openCount - 1;
    closed(cur) = true;
    nodesExpanded = nodesExpanded + 1;
    if cur == gIdx, found = true; break; end

    [cy, cx] = ind2sub([rows cols], cur);
    for m = 1:8
        nx = cx + moves(m, 1);
        ny = cy + moves(m, 2);
        if nx < 1 || nx > cols || ny < 1 || ny > rows, continue; end
        if occ(ny, nx) || closed(ny, nx), continue; end
        % Block diagonal corner cutting between two obstacles/walls
        if moves(m,1) ~= 0 && moves(m,2) ~= 0 && (occ(cy, nx) || occ(ny, cx))
            continue
        end
        nIdx = ny + (nx - 1) * rows;
        gNew = g(cur) + moves(m, 3);
        if gNew < g(nIdx)
            if isinf(fOpen(nIdx)), openCount = openCount + 1; end
            g(nIdx) = gNew;
            fOpen(nIdx) = gNew + H(nIdx);
            parent(nIdx) = cur;
        end
    end
    peakOpen = max(peakOpen, openCount);
end

out.found          = found;
out.nodesExpanded  = nodesExpanded;
out.nodesGenerated = nnz(~isinf(g));  % cells that ever got a cost (memory proxy)
out.peakOpen       = peakOpen;
out.explored       = closed;
out.path           = zeros(0, 2);
out.length         = inf;

if found
    idxList = gIdx;
    cur = gIdx;
    while cur ~= sIdx
        cur = double(parent(cur));
        idxList(end + 1) = cur; %#ok<AGROW>
    end
    idxList = flip(idxList(:));
    [py, px] = ind2sub([rows cols], idxList);
    out.path   = [px py];
    out.length = g(gIdx);             % true path length (1 per straight, sqrt(2) per diagonal)
end
end

function H = heuristicGrid(rows, cols, goal, hType)
% Precomputed heuristic for every cell (distance to goal).
[X, Y] = meshgrid(1:cols, 1:rows);
dx = abs(X - goal(1));
dy = abs(Y - goal(2));
switch lower(hType)
    case 'none',      H = zeros(rows, cols);
    case 'euclidean', H = hypot(dx, dy);
    case 'octile',    H = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy);
    case 'manhattan', H = dx + dy;
    otherwise, error('Unknown heuristic "%s".', hType);
end
end

function [occ, start, goal] = makeSolvableMap(n, density, seed)
% Random occupancy grid with the given obstacle density. Re-draws until a
% path exists. Start/goal neighbourhoods are kept free.
start = [2, 2];
goal  = [n - 1, n - 1];
for attempt = 0:50
    rng(seed + 7919 * attempt);
    occ = rand(n) < density;
    occ = clearAround(occ, start);
    occ = clearAround(occ, goal);
    probe = gridSearch(occ, start, goal, 'none');
    if probe.found, return; end
end
error('Could not generate a solvable %dx%d map at density %.2f.', n, n, density);
end

function occ = clearAround(occ, p)
[rows, cols] = size(occ);
occ(max(1, p(2)-1):min(rows, p(2)+1), max(1, p(1)-1):min(cols, p(1)+1)) = false;
end

function plotSearchOverlay(occ, dij, ast)
% Colour-coded map of which cells each planner expanded.
[rows, cols] = size(occ);
img = ones(rows, cols, 3);                          % free = white
paint = @(img, mask, rgb) setColour(img, mask, rgb);
img = paint(img, dij.explored & ~ast.explored, [0.65 0.85 1.00]);  % Dijkstra only
img = paint(img, dij.explored &  ast.explored, [1.00 0.70 0.35]);  % both
img = paint(img, ast.explored & ~dij.explored, [1.00 0.95 0.40]);  % A* only
img = paint(img, occ, [0 0 0]);                                    % obstacles
image([1 cols], [1 rows], img);
axis xy equal tight;
xlabel('X'); ylabel('Y');
end

function img = setColour(img, mask, rgb)
for c = 1:3
    layer = img(:, :, c);
    layer(mask) = rgb(c);
    img(:, :, c) = layer;
end
end
