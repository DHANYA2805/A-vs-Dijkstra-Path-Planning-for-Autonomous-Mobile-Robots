%% QUICK DEMO: A* vs Dijkstra on a single grid
% Minimal working proof-of-concept for Review 1.
% Self-contained: no toolboxes required (pure MATLAB A* implementation;
% Dijkstra = A* with zero heuristic weight).

clear; clc; close all;

%% 1. Build a small occupancy grid (20x20)
gridSize = 20;
occ = false(gridSize, gridSize);   % occ(y, x) = true means occupied

% Add a simple wall-like obstacle cluster (edit these freely) — [x y] pairs
obstacles = [8 5; 8 6; 8 7; 8 8; 8 9; 8 10; ...
             12 12; 12 13; 12 14; 12 15];
for k = 1:size(obstacles, 1)
    occ(obstacles(k,2), obstacles(k,1)) = true;
end

start = [2, 2];
goal  = [18, 18];

%% 2. Run A*
tic;
[pathAStar, nodesAStar] = gridSearch(occ, start, goal, 1);
timeAStar = toc;

%% 3. Run Dijkstra (A* with zero heuristic = Dijkstra)
tic;
[pathDij, nodesDij] = gridSearch(occ, start, goal, 0);
timeDij = toc;

%% 4. Print comparison table
fprintf('\n--- Comparison on %dx%d grid ---\n', gridSize, gridSize);
fprintf('%-12s %-12s %-15s %-15s\n', 'Algorithm', 'Time (s)', 'Path Points', 'Nodes Explored');
fprintf('%-12s %-12.5f %-15d %-15d\n', 'A*', timeAStar, size(pathAStar,1), nodesAStar);
fprintf('%-12s %-12.5f %-15d %-15d\n', 'Dijkstra', timeDij, size(pathDij,1), nodesDij);

%% 5. Visualize both paths side-by-side
figure('Position', [100 100 1000 450]);

subplot(1,2,1);
plotGrid(occ); hold on;
plot(pathAStar(:,1), pathAStar(:,2), 'g-', 'LineWidth', 2);
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b');
plot(goal(1), goal(2), 'ro', 'MarkerFaceColor', 'r');
title('A* Path');

subplot(1,2,2);
plotGrid(occ); hold on;
plot(pathDij(:,1), pathDij(:,2), 'm-', 'LineWidth', 2);
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b');
plot(goal(1), goal(2), 'ro', 'MarkerFaceColor', 'r');
title('Dijkstra Path');

sgtitle('A* vs Dijkstra — Single Scenario Demo');

%% Local functions
function [path, nodesExplored] = gridSearch(occ, start, goal, hWeight)
% A* on an 8-connected grid. hWeight = 1 gives A*, hWeight = 0 gives Dijkstra.
% start/goal are [x y]; path is an N-by-2 list of [x y] points.
[rows, cols] = size(occ);
idx = @(x, y) sub2ind([rows cols], y, x);

g = inf(rows, cols);            % cost from start
f = inf(rows, cols);            % g + weighted heuristic
parent = zeros(rows, cols);     % linear index of predecessor
inOpen = false(rows, cols);
closed = false(rows, cols);

heur = @(x, y) hWeight * hypot(goal(1) - x, goal(2) - y);

g(start(2), start(1)) = 0;
f(start(2), start(1)) = heur(start(1), start(2));
inOpen(start(2), start(1)) = true;
nodesExplored = 0;

moves = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; ...
         1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

while any(inOpen(:))
    % Pop the open node with the lowest f
    fOpen = f;
    fOpen(~inOpen) = inf;
    [~, cur] = min(fOpen(:));
    [cy, cx] = ind2sub([rows cols], cur);
    inOpen(cur) = false;
    closed(cur) = true;
    nodesExplored = nodesExplored + 1;

    if cx == goal(1) && cy == goal(2)
        break
    end

    for m = 1:size(moves, 1)
        nx = cx + moves(m, 1);
        ny = cy + moves(m, 2);
        if nx < 1 || nx > cols || ny < 1 || ny > rows
            continue
        end
        if occ(ny, nx) || closed(ny, nx)
            continue
        end
        gNew = g(cy, cx) + moves(m, 3);
        if gNew < g(ny, nx)
            g(ny, nx) = gNew;
            f(ny, nx) = gNew + heur(nx, ny);
            parent(ny, nx) = cur;
            inOpen(ny, nx) = true;
        end
    end
end

if isinf(g(goal(2), goal(1)))
    error('No path found from start to goal.');
end

% Reconstruct path by walking parents back from the goal
path = goal;
cur = idx(goal(1), goal(2));
while parent(cur) ~= 0
    cur = parent(cur);
    [py, px] = ind2sub([rows cols], cur);
    path = [px py; path]; %#ok<AGROW>
end
end

function plotGrid(occ)
% Draw the occupancy grid: occupied cells black, free cells white.
[rows, cols] = size(occ);
imagesc([1 cols], [1 rows], ~occ);
colormap(gray);
axis xy equal tight;
xlabel('X'); ylabel('Y');
end
