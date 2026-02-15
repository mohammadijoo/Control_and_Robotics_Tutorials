% Simple directed regrasp graph and shortest path in MATLAB

% Node indices for candidate grasps
nodes = 1:4;

% Edges: from, to, cost
E_from = [1 2 1 3];
E_to   = [2 3 4 4];
E_cost = [1.0 1.0 1.0 2.5];

G = digraph(E_from, E_to, E_cost, numel(nodes));
[startNode, goalNode] = deal(1, 4);

[path, totalCost] = shortestpath(G, startNode, goalNode, "Method", "positive");
disp("Regrasp path:");
disp(path);
disp("Total cost:");
disp(totalCost);

% In a full implementation, each node would carry the hand configuration q_h
% and object pose T_ho, and feasibility of edges would be checked using
% Robotics System Toolbox (e.g., rigidBodyTree, inverseKinematics).
      
