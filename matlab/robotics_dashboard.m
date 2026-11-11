function robotics_dashboard
% ROBOTICS_DASHBOARD  Offline interactive dashboard for the project
%   "Comparative Analysis of A* and Dijkstra Algorithms for Autonomous
%    Mobile Robot Path Planning Using MATLAB" (BECE312L)
%
% Run it by typing  robotics_dashboard  in the MATLAB command window.
% Works fully offline: core MATLAB only (R2020b or newer), no toolboxes,
% no internet. All results are computed live inside the app.
%
% Tabs
%   Overview     - key numbers + decision guideline (Figure 6)
%   Heuristics   - % fewer nodes per heuristic (Figure 1)
%   Manhattan    - why Manhattan is not admissible (Figure 2)
%   Open list    - binary heap vs linear scan (Figure 3)
%   Robot motion - smoothed path, wheel speeds, tracking error, animation (Figures 4 & 5)
%   Live planner - generate any map and compare Dijkstra vs A* on the spot
%   Data         - full results table with CSV export

%% ---------------- settings & shared state ----------------
S.mapSizes  = [20 50 100];
S.densities = [0.10 0.20 0.30];
S.densNames = {'Sparse', 'Medium', 'Dense'};
S.nTrials   = 5;
S.reps      = 3;
S.baseSeed  = 2026;
S.heur      = {'none', 'euclidean', 'octile', 'manhattan'};
S.hLab      = {'Dijkstra', 'A* Euclidean', 'A* Octile', 'A* Manhattan'};
R = struct('ready', false);
animRunning = false;
NAVY = [0.13 0.16 0.36]; MINT = [0.56 0.83 0.78];
hCols = [0.27 0.45 0.77; 0.35 0.62 0.48; 0.80 0.42 0.20];

%% ---------------- main window ----------------
fig = uifigure('Name', 'A* vs Dijkstra - Path Planning Dashboard', ...
               'Position', [40 40 1380 840]);
try, fig.Theme = 'light'; catch, end
root = uigridlayout(fig, [2 1]);
root.RowHeight = {72, '1x'}; root.Padding = [8 8 8 8]; root.RowSpacing = 8;

hdr = uigridlayout(root, [2 3]);
hdr.ColumnWidth = {'1x', 200, 300}; hdr.RowHeight = {'1x', '1x'};
hdr.Padding = [14 6 14 6]; hdr.BackgroundColor = NAVY;
lt = uilabel(hdr, 'Text', 'Autonomous Mobile Robot Path Planning: A* vs Dijkstra', ...
    'FontSize', 21, 'FontWeight', 'bold', 'FontColor', [1 1 1]);
lt.Layout.Row = 1; lt.Layout.Column = 1;
ls = uilabel(hdr, 'Text', 'Robotics and Automation (BECE312L)   |   Nithujaa J  ·  Dhanya S  ·  Sri Nithi Sathiyamoorthy', ...
    'FontSize', 13, 'FontColor', MINT);
ls.Layout.Row = 2; ls.Layout.Column = 1;
btnRun = uibutton(hdr, 'Text', 'Run full benchmark', 'FontSize', 14, 'FontWeight', 'bold', ...
    'ButtonPushedFcn', @(~, ~) runBenchmark());
btnRun.Layout.Row = [1 2]; btnRun.Layout.Column = 2;
lblStatus = uilabel(hdr, 'Text', 'Press "Run full benchmark" to compute all results (about 30-60 s).', ...
    'FontColor', [1 1 1], 'WordWrap', 'on');
lblStatus.Layout.Row = [1 2]; lblStatus.Layout.Column = 3;

tg = uitabgroup(root);

%% ---------------- Tab: Overview ----------------
tabOv = uitab(tg, 'Title', 'Overview');
gOv = uigridlayout(tabOv, [2 2]);
gOv.RowHeight = {120, '1x'}; gOv.ColumnWidth = {'3x', '2x'};
kpiGrid = uigridlayout(gOv, [1 6]); kpiGrid.Layout.Row = 1; kpiGrid.Layout.Column = [1 2];
kpiGrid.Padding = [0 0 0 0];
kpi = gobjects(1, 6);
kpiTitles = {'Maps tested', 'A* optimal (vs Dijkstra)', 'Best heuristic', ...
             'Max search saved', 'Heap speed-up', 'Robot reached goal'};
for kk = 1:6
    kpi(kk) = makeKpi(kpiGrid, kpiTitles{kk});
end
axGuide = uiaxes(gOv); axGuide.Layout.Row = 2; axGuide.Layout.Column = 1;
title(axGuide, 'Figure 6 - Decision guideline');
txtSummary = uitextarea(gOv, 'Editable', 'off', 'FontSize', 14, ...
    'Value', {'Run the benchmark to see the findings here.'});
txtSummary.Layout.Row = 2; txtSummary.Layout.Column = 2;

%% ---------------- Tab: Heuristics (Fig 1) ----------------
tabH = uitab(tg, 'Title', 'Fig 1 - Heuristics');
gH = uigridlayout(tabH, [2 3]); gH.RowHeight = {'1x', 60};
axH = gobjects(1, 3);
for kk = 1:3
    axH(kk) = uiaxes(gH); axH(kk).Layout.Row = 1; axH(kk).Layout.Column = kk;
end
noteH = uilabel(gH, 'WordWrap', 'on', 'FontSize', 13, 'Text', ...
    ['Bars show how many fewer cells each A* heuristic explored than Dijkstra (higher = less work). ', ...
     'Octile beats Euclidean everywhere and keeps the shortest path; Manhattan saves the most only by giving up optimality.']);
noteH.Layout.Row = 2; noteH.Layout.Column = [1 3];

%% ---------------- Tab: Manhattan (Fig 2) ----------------
tabM = uitab(tg, 'Title', 'Fig 2 - Manhattan');
gM = uigridlayout(tabM, [2 2]); gM.RowHeight = {'1x', 60};
axM1 = uiaxes(gM); axM1.Layout.Row = 1; axM1.Layout.Column = 1;
axM2 = uiaxes(gM); axM2.Layout.Row = 1; axM2.Layout.Column = 2;
noteM = uilabel(gM, 'WordWrap', 'on', 'FontSize', 13, 'Text', ...
    ['A diagonal step really costs 1.41, but Manhattan counts it as 2. Because it overestimates, ', ...
     'A* with Manhattan can miss the shortest path. Left: how much longer the path is. Right: how often that happens.']);
noteM.Layout.Row = 2; noteM.Layout.Column = [1 2];

%% ---------------- Tab: Open list (Fig 3) ----------------
tabO = uitab(tg, 'Title', 'Fig 3 - Open list');
gO = uigridlayout(tabO, [2 2]); gO.RowHeight = {'1x', 60}; gO.ColumnWidth = {'2x', '1x'};
axO = uiaxes(gO); axO.Layout.Row = 1; axO.Layout.Column = 1;
tblO = uitable(gO); tblO.Layout.Row = 1; tblO.Layout.Column = 2;
noteO = uilabel(gO, 'WordWrap', 'on', 'FontSize', 13, 'Text', ...
    ['Same algorithms, same paths - only the data structure that picks the next cell changes. ', ...
     'A binary heap finds the lowest-cost cell instantly instead of scanning the whole map (100x100 maps, log scale).']);
noteO.Layout.Row = 2; noteO.Layout.Column = [1 2];

%% ---------------- Tab: Robot motion (Fig 4 & 5) ----------------
tabR = uitab(tg, 'Title', 'Fig 4 & 5 - Robot motion');
gR = uigridlayout(tabR, [3 2]); gR.RowHeight = {'1x', '1x', 130}; gR.ColumnWidth = {'1.2x', '1x'};
axMap = uiaxes(gR); axMap.Layout.Row = [1 2]; axMap.Layout.Column = 1;
axWheel = uiaxes(gR); axWheel.Layout.Row = 1; axWheel.Layout.Column = 2;
axCte = uiaxes(gR); axCte.Layout.Row = 2; axCte.Layout.Column = 2;
gRb = uigridlayout(gR, [2 1]); gRb.Layout.Row = 3; gRb.Layout.Column = 1; gRb.Padding = [0 0 0 0];
btnAnim = uibutton(gRb, 'Text', 'Animate robot', 'FontSize', 14, 'FontWeight', 'bold', ...
    'ButtonPushedFcn', @(~, ~) toggleAnimation());
uilabel(gRb, 'WordWrap', 'on', 'Text', ...
    'Grey dotted: raw A* path. Blue: smoothed path. Red: route actually driven by the two-wheeled robot (pure-pursuit steering).');
tblR = uitable(gR); tblR.Layout.Row = 3; tblR.Layout.Column = 2;

%% ---------------- Tab: Live planner ----------------
tabL = uitab(tg, 'Title', 'Live planner');
gL = uigridlayout(tabL, [1 2]); gL.ColumnWidth = {280, '1x'};
ctrl = uigridlayout(gL, [12 1]);
ctrl.RowHeight = {22, 30, 22, 30, 22, 30, 22, 30, 40, 40, '1x', 60};
uilabel(ctrl, 'Text', 'Map size', 'FontWeight', 'bold');
ddSize = uidropdown(ctrl, 'Items', {'20 x 20', '50 x 50', '100 x 100', '150 x 150'}, ...
    'ItemsData', [20 50 100 150], 'Value', 50);
uilabel(ctrl, 'Text', 'Obstacle density', 'FontWeight', 'bold');
ddDens = uidropdown(ctrl, 'Items', {'Sparse (10%)', 'Medium (20%)', 'Dense (30%)', 'Very dense (35%)'}, ...
    'ItemsData', [0.10 0.20 0.30 0.35], 'Value', 0.20);
uilabel(ctrl, 'Text', 'Random seed', 'FontWeight', 'bold');
spSeed = uispinner(ctrl, 'Limits', [1 99999], 'Value', 1, 'RoundFractionalValues', 'on');
uilabel(ctrl, 'Text', 'A* heuristic', 'FontWeight', 'bold');
ddHeur = uidropdown(ctrl, 'Items', {'Octile', 'Euclidean', 'Manhattan'}, ...
    'ItemsData', {'octile', 'euclidean', 'manhattan'}, 'Value', 'octile');
uibutton(ctrl, 'Text', 'Plan on this map', 'FontSize', 14, 'FontWeight', 'bold', ...
    'ButtonPushedFcn', @(~, ~) livePlan());
uibutton(ctrl, 'Text', 'New random map', 'FontSize', 14, ...
    'ButtonPushedFcn', @(~, ~) newRandomMap());
tblL = uitable(ctrl);
lblLive = uilabel(ctrl, 'Text', '', 'WordWrap', 'on', 'FontWeight', 'bold', 'FontSize', 13);
gLa = uigridlayout(gL, [1 2]);
axLD = uiaxes(gLa); axLA = uiaxes(gLa);

%% ---------------- Tab: Data ----------------
tabD = uitab(tg, 'Title', 'Data');
gD = uigridlayout(tabD, [2 1]); gD.RowHeight = {'1x', 40};
tblD = uitable(gD);
uibutton(gD, 'Text', 'Export results to CSV', 'ButtonPushedFcn', @(~, ~) exportCsv());

livePlan();   % show a first live example straight away

%% ======================= callbacks (nested) =======================

    function runBenchmark()
        btnRun.Enable = 'off';
        nS = numel(S.mapSizes); nD = numel(S.densities); nH = numel(S.heur);
        total = nS * nD * S.nTrials;
        dlg = uiprogressdlg(fig, 'Title', 'Running benchmark', 'Message', 'Generating maps...');
        cleanupObj = onCleanup(@() set(btnRun, 'Enable', 'on')); %#ok<NASGU>
        nodes = zeros(nS, nD, nH); tms = nodes; plen = nodes;
        subMean = nodes; subMax = nodes; subFrac = nodes;
        maps = cell(nS, nD, S.nTrials);
        rows = {}; done = 0; optimalCount = 0;
        for si = 1:nS
            for di = 1:nD
                acc = zeros(S.nTrials, nH, 4);   % nodes, time, length, excess
                for t = 1:S.nTrials
                    seed = S.baseSeed + 1000 * si + 100 * di + t;
                    [occ, st, gl] = makeSolvableMap(S.mapSizes(si), S.densities(di), seed);
                    maps{si, di, t} = struct('occ', occ, 'start', st, 'goal', gl);
                    optLen = NaN;
                    for hi = 1:nH
                        [out, tMs] = timedSearch(occ, st, gl, S.heur{hi}, 'heap', S.reps);
                        if hi == 1, optLen = out.length; end
                        exc = 100 * (out.length / optLen - 1);
                        acc(t, hi, :) = [out.nodesExpanded, tMs, out.length, exc];
                        rows(end + 1, :) = {S.mapSizes(si), S.densNames{di}, t, S.hLab{hi}, ...
                            out.nodesExpanded, tMs, out.length, exc}; %#ok<AGROW>
                    end
                    if abs(acc(t, 3, 4)) < 1e-6, optimalCount = optimalCount + 1; end
                    done = done + 1;
                    dlg.Value = 0.75 * done / total;
                    dlg.Message = sprintf('Heuristic benchmark: map %d of %d', done, total);
                end
                nodes(si, di, :) = mean(acc(:, :, 1), 1);
                tms(si, di, :)   = mean(acc(:, :, 2), 1);
                plen(si, di, :)  = mean(acc(:, :, 3), 1);
                subMean(si, di, :) = mean(acc(:, :, 4), 1);
                subMax(si, di, :)  = max(acc(:, :, 4), [], 1);
                subFrac(si, di, :) = 100 * mean(acc(:, :, 4) > 1e-6, 1);
            end
        end

        % Open list comparison on the largest maps
        dlg.Message = 'Comparing heap vs linear open list on 100x100 maps...';
        ol = zeros(nD, 4);   % Dij-linear, Dij-heap, A*-linear, A*-heap
        for di = 1:nD
            tAcc = zeros(S.nTrials, 4);
            for t = 1:S.nTrials
                Mp = maps{nS, di, t};
                [~, tAcc(t, 1)] = timedSearch(Mp.occ, Mp.start, Mp.goal, 'none', 'linear', 1);
                [~, tAcc(t, 2)] = timedSearch(Mp.occ, Mp.start, Mp.goal, 'none', 'heap', S.reps);
                [~, tAcc(t, 3)] = timedSearch(Mp.occ, Mp.start, Mp.goal, 'euclidean', 'linear', 1);
                [~, tAcc(t, 4)] = timedSearch(Mp.occ, Mp.start, Mp.goal, 'euclidean', 'heap', S.reps);
            end
            ol(di, :) = mean(tAcc, 1);
            dlg.Value = 0.75 + 0.15 * di / nD;
        end

        % Robot motion on the 50x50 medium map
        dlg.Message = 'Simulating the robot...'; dlg.Value = 0.95;
        Mp = maps{2, 2, 1};
        pl = gridSearch(Mp.occ, Mp.start, Mp.goal, 'octile', 'heap');
        raw = pl.path; smo = smoothPath(raw, Mp.occ, 0.35);
        rb = struct('v', 1.0, 'L', 1.5, 'wMax', 2.5, 'b', 0.5, 'dt', 0.05, ...
                    'goalTol', 0.3, 'maxTime', 3 * pl.length + 20);
        simRaw = simulateRobot(raw, Mp.occ, rb);
        simSmo = simulateRobot(smo, Mp.occ, rb);

        % Decision guideline (A* with Octile)
        speed = tms(:, :, 1) ./ tms(:, :, 3);
        red   = 100 * (1 - nodes(:, :, 3) ./ nodes(:, :, 1));
        rec = 2 * ones(nS, nD);
        rec(tms(:, :, 1) < 1) = 1;
        rec(tms(:, :, 1) >= 1 & speed >= 2) = 3;

        R = struct('ready', true, 'nodes', nodes, 'tms', tms, 'plen', plen, ...
            'subMean', subMean, 'subMax', subMax, 'subFrac', subFrac, ...
            'ol', ol, 'map', Mp, 'raw', raw, 'smo', smo, 'simRaw', simRaw, 'simSmo', simSmo, ...
            'speed', speed, 'red', red, 'rec', rec, 'optimalCount', optimalCount, 'total', total, ...
            'raw_rows', {rows});
        dlg.Value = 1; close(dlg);
        drawAll();
        lblStatus.Text = sprintf('Benchmark finished: %d maps, %d planners. All tabs updated.', total, nH);
    end

    function drawAll()
        drawOverview(); drawHeuristics(); drawManhattan(); drawOpenList(); drawRobot(); drawData();
    end

    function drawOverview()
        ndAll = R.nodes;
        kpi(1).Text = sprintf('%d', R.total);
        kpi(2).Text = sprintf('%d / %d', R.optimalCount, R.total);
        kpi(3).Text = 'Octile';
        kpi(4).Text = sprintf('%.0f%%', max(R.red(:)));
        kpi(5).Text = sprintf('%.0fx', mean([R.ol(:, 1) ./ R.ol(:, 2); R.ol(:, 3) ./ R.ol(:, 4)]));
        if R.simSmo.reached && ~R.simSmo.collided
            kpi(6).Text = 'Yes';
        elseif R.simSmo.reached
            kpi(6).Text = 'Yes*';
        else
            kpi(6).Text = 'No';
        end

        cla(axGuide);
        imagesc(axGuide, R.rec, [1 3]);
        colormap(axGuide, [0.80 0.90 0.95; 0.53 0.73 0.85; 0.13 0.35 0.55]);
        names = {'Either is fine', 'A* recommended', 'A* strongly recommended'};
        for si = 1:3
            for di = 1:3
                c = [0 0 0]; if R.rec(si, di) == 3, c = [1 1 1]; end
                text(axGuide, di, si, sprintf('%s\n%.1fx faster, %.0f%% fewer nodes', ...
                    names{R.rec(si, di)}, R.speed(si, di), R.red(si, di)), ...
                    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', c, 'FontSize', 11);
            end
        end
        axGuide.XTick = 1:3; axGuide.XTickLabel = S.densNames;
        axGuide.YTick = 1:3; axGuide.YTickLabel = {'20x20', '50x50', '100x100'};
        xlabel(axGuide, 'Obstacle density'); ylabel(axGuide, 'Map size');
        title(axGuide, 'Figure 6 - Decision guideline (A* with Octile heuristic)');
        axis(axGuide, 'tight');

        manhFrac = mean(R.subFrac(:, :, 4), 'all');
        txtSummary.Value = {
            'KEY FINDINGS'
            ''
            sprintf('1. A* with Octile found the same shortest path as Dijkstra on %d of %d maps.', R.optimalCount, R.total)
            sprintf('2. Octile is the best heuristic: %.0f cells on average vs %.0f for Euclidean.', ...
                mean(ndAll(:, :, 3), 'all'), mean(ndAll(:, :, 2), 'all'))
            sprintf('3. Manhattan is not optimal on %.0f%% of maps (up to %.2f%% longer paths).', ...
                manhFrac, max(R.subMax(:, :, 4), [], 'all'))
            sprintf('4. On large open maps A* is %.1fx faster with %.0f%% fewer cells.', R.speed(3, 1), R.red(3, 1))
            '5. On small maps both plan in under 1 ms - either algorithm is fine.'
            sprintf('6. A binary-heap open list is about %s faster with identical paths.', kpi(5).Text)
            sprintf('7. The simulated robot reached the goal in %.1f s, staying within %.2f cells of the path.', ...
                R.simSmo.time, R.simSmo.maxCTE)
            ''
            'CONCLUSION: use A* with the Octile heuristic for 8-direction grid robots;'
            'its benefit grows with map size and openness.'};
    end

    function drawHeuristics()
        for di = 1:3
            ax = axH(di); cla(ax);
            redH = 100 * (1 - squeeze(R.nodes(:, di, 2:4)) ./ R.nodes(:, di, 1));
            b = bar(ax, redH, 'grouped');
            for k = 1:3, b(k).FaceColor = hCols(k, :); end
            ax.XTick = 1:3; ax.XTickLabel = {'20x20', '50x50', '100x100'};
            ylim(ax, [0 100]); grid(ax, 'on');
            xlabel(ax, 'Map size'); ylabel(ax, '% fewer nodes than Dijkstra');
            title(ax, sprintf('%s obstacles (%d%%)', S.densNames{di}, round(100 * S.densities(di))));
            if di == 1, legend(ax, S.hLab(2:4), 'Location', 'southwest'); end
        end
    end

    function drawManhattan()
        cla(axM1); cla(axM2);
        bar(axM1, R.subMean(:, :, 4)); grid(axM1, 'on');
        axM1.XTick = 1:3; axM1.XTickLabel = {'20x20', '50x50', '100x100'};
        xlabel(axM1, 'Map size'); ylabel(axM1, 'Mean path excess over optimum (%)');
        title(axM1, 'Manhattan: path length penalty'); legend(axM1, S.densNames, 'Location', 'northwest');
        bar(axM2, R.subFrac(:, :, 4)); grid(axM2, 'on'); ylim(axM2, [0 105]);
        axM2.XTick = 1:3; axM2.XTickLabel = {'20x20', '50x50', '100x100'};
        xlabel(axM2, 'Map size'); ylabel(axM2, '% of maps with non-optimal path');
        title(axM2, 'Manhattan: how often it is not optimal');
    end

    function drawOpenList()
        cla(axO);
        b = bar(axO, R.ol); grid(axO, 'on'); axO.YScale = 'log';
        cols = [0.27 0.45 0.77; 0.78 0.35 0.10; 0.87 0.68 0.20; 0.47 0.20 0.78];
        for k = 1:4, b(k).FaceColor = cols(k, :); end
        axO.XTick = 1:3; axO.XTickLabel = S.densNames;
        xlabel(axO, 'Obstacle density (100x100 maps)'); ylabel(axO, 'Planning time (ms, log scale)');
        title(axO, 'Binary-heap open list vs linear scan');
        legend(axO, {'Dijkstra - linear', 'Dijkstra - heap', 'A* - linear', 'A* - heap'}, 'Location', 'northeast');
        tblO.Data = table(string(S.densNames'), round(R.ol(:, 1) ./ R.ol(:, 2), 1), ...
            round(R.ol(:, 3) ./ R.ol(:, 4), 1), 'VariableNames', {'Density', 'Dijkstra gain (x)', 'A* gain (x)'});
    end

    function drawRobot()
        Mp = R.map;
        cla(axMap); hold(axMap, 'on');
        showMap(axMap, Mp.occ);
        h1 = plot(axMap, R.raw(:, 1), R.raw(:, 2), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
        h2 = plot(axMap, R.smo(:, 1), R.smo(:, 2), 'b--o', 'LineWidth', 1.5, 'MarkerSize', 4);
        h3 = plot(axMap, R.simSmo.traj(:, 1), R.simSmo.traj(:, 2), 'r-', 'LineWidth', 2);
        plot(axMap, Mp.start(1), Mp.start(2), 'bo', 'MarkerFaceColor', 'b');
        plot(axMap, Mp.goal(1), Mp.goal(2), 'ro', 'MarkerFaceColor', 'r');
        hold(axMap, 'off');
        legend(axMap, [h1 h2 h3], {'Raw A* path', 'Smoothed path', 'Robot trajectory'}, 'Location', 'southeast');
        title(axMap, sprintf('Robot following smoothed A* path (50x50, medium): %d -> %d waypoints', ...
            size(R.raw, 1), size(R.smo, 1)));

        cla(axWheel);
        plot(axWheel, R.simSmo.t, R.simSmo.vR, 'LineWidth', 1.5); hold(axWheel, 'on');
        plot(axWheel, R.simSmo.t, R.simSmo.vL, 'LineWidth', 1.5); hold(axWheel, 'off');
        grid(axWheel, 'on'); xlabel(axWheel, 'Time (s)'); ylabel(axWheel, 'Wheel speed (cells/s)');
        legend(axWheel, {'Right wheel', 'Left wheel'}); title(axWheel, 'Wheel speeds (differential drive)');

        cla(axCte);
        plot(axCte, R.simRaw.t, R.simRaw.cte, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2); hold(axCte, 'on');
        plot(axCte, R.simSmo.t, R.simSmo.cte, 'r', 'LineWidth', 1.4); hold(axCte, 'off');
        grid(axCte, 'on'); xlabel(axCte, 'Time (s)'); ylabel(axCte, 'Tracking error (cells)');
        legend(axCte, {'Raw path', 'Smoothed path'}); title(axCte, 'Distance from planned path');

        sims = {R.simRaw, R.simSmo};
        tblR.Data = table(["Raw A* path"; "Smoothed path"], ...
            [size(R.raw, 1); size(R.smo, 1)], ...
            round([sims{1}.time; sims{2}.time], 1), ...
            round([sims{1}.maxCTE; sims{2}.maxCTE], 2), ...
            round([sims{1}.minClear; sims{2}.minClear], 2), ...
            [sims{1}.reached; sims{2}.reached], [sims{1}.collided; sims{2}.collided], ...
            'VariableNames', {'Path', 'Waypoints', 'Time (s)', 'Max error', 'Min clearance', 'Reached', 'Collided'});
    end

    function toggleAnimation()
        if ~R.ready
            uialert(fig, 'Run the full benchmark first.', 'No results yet'); return
        end
        if animRunning
            animRunning = false; return
        end
        animRunning = true; btnAnim.Text = 'Stop animation';
        Mp = R.map; sim = R.simSmo;
        cla(axMap); hold(axMap, 'on');
        showMap(axMap, Mp.occ);
        plot(axMap, R.smo(:, 1), R.smo(:, 2), 'b--', 'LineWidth', 1.5);
        plot(axMap, Mp.start(1), Mp.start(2), 'bo', 'MarkerFaceColor', 'b');
        plot(axMap, Mp.goal(1), Mp.goal(2), 'ro', 'MarkerFaceColor', 'r');
        trail = plot(axMap, nan, nan, 'r-', 'LineWidth', 2.5);
        body  = patch(axMap, nan, nan, [0.1 0.6 0.3], 'EdgeColor', 'k');
        hold(axMap, 'off');
        title(axMap, 'Figure 5 - Differential-drive robot following the smoothed A* path');
        shape = [0.9 0; -0.6 0.5; -0.6 -0.5];
        nK = size(sim.traj, 1);
        for k = [1:2:nK, nK]
            if ~animRunning || ~isvalid(fig), break; end
            th = sim.traj(k, 3);
            Rm = [cos(th) -sin(th); sin(th) cos(th)];
            pts = (Rm * shape')' + sim.traj(k, 1:2);
            set(body, 'XData', pts(:, 1), 'YData', pts(:, 2));
            set(trail, 'XData', sim.traj(1:k, 1), 'YData', sim.traj(1:k, 2));
            drawnow limitrate;
            pause(0.01);
        end
        animRunning = false;
        if isvalid(fig)
            btnAnim.Text = 'Animate robot';
            drawRobot();
        end
    end

    function newRandomMap()
        spSeed.Value = randi([1 99999]);
        livePlan();
    end

    function livePlan()
        n = ddSize.Value; dens = ddDens.Value; seed = spSeed.Value; h = ddHeur.Value;
        try
            [occ, st, gl] = makeSolvableMap(n, dens, seed);
        catch ME
            uialert(fig, ME.message, 'Map generation failed'); return
        end
        [dij, tD] = timedSearch(occ, st, gl, 'none', 'heap', 3);
        [ast, tA] = timedSearch(occ, st, gl, h, 'heap', 3);
        drawExplored(axLD, occ, dij, st, gl, [0.65 0.85 1.00], 'm', ...
            sprintf('Dijkstra: %d cells explored', dij.nodesExpanded));
        drawExplored(axLA, occ, ast, st, gl, [1.00 0.70 0.35], [0 0.6 0], ...
            sprintf('A* (%s): %d cells explored', ddHeur.Items{strcmp(ddHeur.ItemsData, h)}, ast.nodesExpanded));
        tblL.Data = table(["Dijkstra"; "A*"], [dij.nodesExpanded; ast.nodesExpanded], ...
            round([tD; tA], 2), round([dij.length; ast.length], 2), ...
            'VariableNames', {'Planner', 'Cells', 'Time (ms)', 'Length'});
        msg = sprintf('A* explored %.0f%% fewer cells.', 100 * (1 - ast.nodesExpanded / dij.nodesExpanded));
        if ast.length > dij.length + 1e-6
            msg = sprintf('%s Its path is %.2f%% LONGER (heuristic not admissible).', msg, ...
                100 * (ast.length / dij.length - 1));
        else
            msg = [msg ' Same shortest path as Dijkstra.'];
        end
        lblLive.Text = msg;
    end

    function drawData()
        rr = R.raw_rows;
        T = cell2table(rr, 'VariableNames', {'MapSize', 'Density', 'Trial', 'Planner', ...
            'NodesExpanded', 'TimeMs', 'PathLength', 'PathExcessPct'});
        T.TimeMs = round(T.TimeMs, 3); T.PathLength = round(T.PathLength, 2);
        T.PathExcessPct = round(T.PathExcessPct, 2);
        tblD.Data = T;
    end

    function exportCsv()
        if ~R.ready
            uialert(fig, 'Run the full benchmark first.', 'No results yet'); return
        end
        [f, p] = uiputfile('dashboard_results.csv', 'Save results');
        if isequal(f, 0), return; end
        writetable(tblD.Data, fullfile(p, f));
        lblStatus.Text = ['Saved ' fullfile(p, f)];
    end

    function lbl = makeKpi(parent, ttl)
        pnl = uipanel(parent, 'BackgroundColor', [1 1 1]);
        g = uigridlayout(pnl, [2 1]); g.RowHeight = {'2x', '1x'}; g.BackgroundColor = [1 1 1];
        lbl = uilabel(g, 'Text', '-', 'FontSize', 30, 'FontWeight', 'bold', ...
            'FontColor', [0.02 0.35 0.51], 'HorizontalAlignment', 'center');
        uilabel(g, 'Text', ttl, 'FontColor', [0.36 0.42 0.46], 'HorizontalAlignment', 'center', ...
            'WordWrap', 'on');
    end
end

%% ======================= helper functions (local) =======================

function drawExplored(ax, occ, out, st, gl, colExpl, colPath, ttl)
cla(ax);
[rows, cols] = size(occ);
img = ones(rows, cols, 3);
for c = 1:3
    layer = img(:, :, c);
    layer(out.explored) = colExpl(c);
    layer(occ) = 0;
    img(:, :, c) = layer;
end
image(ax, [1 cols], [1 rows], img);
hold(ax, 'on');
plot(ax, out.path(:, 1), out.path(:, 2), '-', 'Color', colPath, 'LineWidth', 2.5);
plot(ax, st(1), st(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
plot(ax, gl(1), gl(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
hold(ax, 'off');
ax.YDir = 'normal'; axis(ax, 'equal'); axis(ax, 'tight');
title(ax, ttl);
end

function showMap(ax, occ)
[rows, cols] = size(occ);
image(ax, [1 cols], [1 rows], repmat(double(~occ), 1, 1, 3));
ax.YDir = 'normal'; axis(ax, 'equal'); axis(ax, 'tight');
xlabel(ax, 'X'); ylabel(ax, 'Y');
end

function out = gridSearch(occ, start, goal, hType, engine)
% 8-connected grid search, no corner cutting.
% hType 'none' = Dijkstra; 'euclidean' | 'octile' | 'manhattan' = A*.
% engine 'heap' (binary heap) or 'linear' (scan whole grid).
if nargin < 5, engine = 'heap'; end
useHeap = strcmpi(engine, 'heap');
[rows, cols] = size(occ); N = rows * cols;
H = heuristicGrid(rows, cols, goal, hType);
g = inf(N, 1); parent = zeros(N, 1, 'uint32');
closed = false(N, 1); inOpen = false(N, 1);
sIdx = sub2ind([rows cols], start(2), start(1));
gIdx = sub2ind([rows cols], goal(2), goal(1));
g(sIdx) = 0;
if useHeap
    hKey = zeros(8 * N + 1, 1); hNode = zeros(8 * N + 1, 1);
    hN = 1; hKey(1) = H(sIdx); hNode(1) = sIdx;
else
    fOpen = inf(N, 1); fOpen(sIdx) = H(sIdx);
end
inOpen(sIdx) = true; nodesExpanded = 0; found = false;
moves = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; 1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];
while true
    if useHeap
        cur = 0;
        while hN > 0
            cand = hNode(1);
            hKey(1) = hKey(hN); hNode(1) = hNode(hN); hN = hN - 1;
            i = 1;
            while true
                l = 2 * i;
                if l > hN, break; end
                c = l;
                if l + 1 <= hN && hKey(l + 1) < hKey(l), c = l + 1; end
                if hKey(i) <= hKey(c), break; end
                tk = hKey(i); hKey(i) = hKey(c); hKey(c) = tk;
                tn = hNode(i); hNode(i) = hNode(c); hNode(c) = tn;
                i = c;
            end
            if ~closed(cand), cur = cand; break; end
        end
        if cur == 0, break; end
    else
        [fMin, cur] = min(fOpen);
        if isinf(fMin), break; end
        fOpen(cur) = inf;
    end
    inOpen(cur) = false; closed(cur) = true; nodesExpanded = nodesExpanded + 1;
    if cur == gIdx, found = true; break; end
    cy = mod(cur - 1, rows) + 1; cx = floor((cur - 1) / rows) + 1;
    for m = 1:8
        nx = cx + moves(m, 1); ny = cy + moves(m, 2);
        if nx < 1 || nx > cols || ny < 1 || ny > rows, continue; end
        nIdx = ny + (nx - 1) * rows;
        if occ(nIdx) || closed(nIdx), continue; end
        if moves(m, 1) ~= 0 && moves(m, 2) ~= 0 && (occ(cy, nx) || occ(ny, cx)), continue; end
        gNew = g(cur) + moves(m, 3);
        if gNew < g(nIdx)
            g(nIdx) = gNew; parent(nIdx) = cur; key = gNew + H(nIdx);
            inOpen(nIdx) = true;
            if useHeap
                hN = hN + 1; i = hN; hKey(i) = key; hNode(i) = nIdx;
                while i > 1
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
end
out.found = found; out.nodesExpanded = nodesExpanded;
out.explored = reshape(closed, rows, cols);
out.path = zeros(0, 2); out.length = inf;
if found
    idx = gIdx; cur = gIdx;
    while cur ~= sIdx
        cur = double(parent(cur)); idx(end + 1) = cur; %#ok<AGROW>
    end
    idx = flip(idx(:));
    [py, px] = ind2sub([rows cols], idx);
    out.path = [px py]; out.length = g(gIdx);
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

function [out, tMs] = timedSearch(occ, st, gl, hType, engine, reps)
times = zeros(reps, 1);
for k = 1:reps
    t0 = tic; out = gridSearch(occ, st, gl, hType, engine); times(k) = toc(t0);
end
tMs = 1000 * median(times);
end

function [occ, start, goal] = makeSolvableMap(n, density, seed)
% Same generator as the project scripts, so the same seeds give the same maps.
start = [2, 2]; goal = [n - 1, n - 1];
for attempt = 0:50
    rng(seed + 7919 * attempt);
    occ = rand(n) < density;
    occ = clearAround(occ, start); occ = clearAround(occ, goal);
    probe = gridSearch(occ, start, goal, 'none', 'heap');
    if probe.found, return; end
end
error('Could not generate a solvable %dx%d map at %.0f%% density - try another seed.', n, n, 100 * density);
end

function occ = clearAround(occ, p)
[rows, cols] = size(occ);
occ(max(1, p(2) - 1):min(rows, p(2) + 1), max(1, p(1) - 1):min(cols, p(1) + 1)) = false;
end

function sp = smoothPath(path, occ, clearance)
n = size(path, 1); sp = path(1, :); i = 1;
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
[rows, cols] = size(occ);
d = q - p; L = hypot(d(1), d(2));
if L == 0, ok = true; return; end
s = linspace(0, 1, max(2, ceil(L / 0.05)))';
pts = p + s .* d; nrm = [-d(2) d(1)] / L; ok = true;
for off = [0, clearance, -clearance]
    q2 = pts + off * nrm;
    cx = round(q2(:, 1)); cy = round(q2(:, 2));
    if any(cx < 1 | cx > cols | cy < 1 | cy > rows), ok = false; return; end
    if any(occ(sub2ind([rows cols], cy, cx))), ok = false; return; end
end
end

function sim = simulateRobot(path, occ, p)
% Differential-drive (unicycle) robot tracking the path with pure pursuit.
ref = resamplePath(path, 0.1); nRef = size(ref, 1);
[rows, cols] = size(occ); [oy, ox] = find(occ);
x = ref(1, 1); y = ref(1, 2);
ah = min(10, nRef); th = atan2(ref(ah, 2) - y, ref(ah, 1) - x);
goal = ref(end, :);
nMax = ceil(p.maxTime / p.dt);
traj = zeros(nMax, 3); vw = zeros(nMax, 2); cte = zeros(nMax, 1); clr = zeros(nMax, 1);
prog = 1; reached = false; collided = false; k = 0;
for k = 1:nMax
    win = prog:min(prog + 60, nRef);
    [dmin, rel] = min(hypot(ref(win, 1) - x, ref(win, 2) - y));
    prog = win(rel);
    la = prog;
    while la < nRef && hypot(ref(la, 1) - x, ref(la, 2) - y) < p.L
        la = la + 1;
    end
    alpha = wrapAngle(atan2(ref(la, 2) - y, ref(la, 1) - x) - th);
    distGoal = hypot(goal(1) - x, goal(2) - y);
    if distGoal < p.goalTol, reached = true; k = k - 1; break; end
    v = p.v * min(1, max(0.2, distGoal / p.L));
    w = max(-p.wMax, min(p.wMax, 2 * v * sin(alpha) / p.L));
    x = x + v * cos(th) * p.dt; y = y + v * sin(th) * p.dt; th = wrapAngle(th + w * p.dt);
    traj(k, :) = [x y th]; vw(k, :) = [v w]; cte(k) = dmin;
    if isempty(ox), clr(k) = inf; else, clr(k) = min(hypot(ox - x, oy - y)) - 0.5; end
    cxr = round(x); cyr = round(y);
    if cxr < 1 || cxr > cols || cyr < 1 || cyr > rows || occ(cyr, cxr), collided = true; end
end
k = max(k, 1);
traj = traj(1:k, :); vw = vw(1:k, :); cte = cte(1:k); clr = clr(1:k);
sim.traj = traj; sim.t = (1:k)' * p.dt;
sim.vR = vw(:, 1) + vw(:, 2) * p.b / 2; sim.vL = vw(:, 1) - vw(:, 2) * p.b / 2;
sim.cte = cte; sim.time = k * p.dt; sim.maxCTE = max(cte); sim.minClear = min(clr);
sim.reached = reached; sim.collided = collided;
end

function rp = resamplePath(path, ds)
seg = hypot(diff(path(:, 1)), diff(path(:, 2)));
path = path([true; seg > 0], :);
s = [0; cumsum(seg(seg > 0))];
sq = (0:ds:s(end))';
if sq(end) < s(end), sq(end + 1) = s(end); end
rp = [interp1(s, path(:, 1), sq), interp1(s, path(:, 2), sq)];
end

function a = wrapAngle(a)
a = mod(a + pi, 2 * pi) - pi;
end
