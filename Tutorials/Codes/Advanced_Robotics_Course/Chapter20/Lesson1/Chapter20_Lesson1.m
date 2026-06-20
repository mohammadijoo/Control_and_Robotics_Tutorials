% Toy adjacency matrix for 5 theoretical robotics papers
% A(i,j) = 1 if paper i cites paper j
A = [
    0 1 1 0 0;
    0 0 1 0 0;
    0 1 0 1 0;
    0 0 0 0 1;
    0 0 0 0 0
];

G = digraph(A);
figure;
plot(G);
title('Citation graph (toy example)');

% Simple out-degree and in-degree
d_out = outdegree(G);
d_in  = indegree(G);

% PageRank-style centrality (R2022b and later)
pr = centrality(G, 'pagerank', 'Importance', G.Edges.Weight);

disp('PageRank centrality:');
disp(pr.');

% Coverage for a chosen seed set S = {1,3} and radius r = 1
seeds = [1 3];
r = 1;
reachable = bfsearch(G, seeds, 'edgetonew');
cov_r1 = numel(unique(reachable)) / numnodes(G);
fprintf('Coverage for seeds {1,3}, r=1: %.3f\n', cov_r1);

% Once a project is selected, you can create a Simulink model shell:
% new_system('CapstoneRobotSystem');
% open_system('CapstoneRobotSystem');
% Use Simulink blocks to represent sensing, planning, control, and execution layers.
      
