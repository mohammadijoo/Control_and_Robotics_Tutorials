% Define vertices as integer indices
s = [1 1 2 2 3];  % source nodes
t = [2 3 3 4 4];  % target nodes
w = [1 2 1 1 0.5]; % edge weights

G = digraph(s, t, w);

start = 1;
goal  = 4;

[path, cost] = shortestpath(G, start, goal);

disp("Path:");
disp(path);
disp("Cost:");
disp(cost);
      
