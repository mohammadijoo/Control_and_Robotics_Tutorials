% Chapter14_Lesson2.m
% Costmaps and Inflation Concepts — build inflated costmap using bwdist.

clear; clc;

resolution = 0.05;      % m/cell
r_inscribed = 0.25;     % m
r_inflation = 0.60;     % m
cost_scaling = 6.0;     % 1/m

w = 60; h = 40;
obs = zeros(h, w);
obs(1,:) = 1; obs(end,:) = 1;
obs(:,1) = 1; obs(:,end) = 1;
obs(13, 11:50) = 1;         % MATLAB is 1-indexed
obs(19:33, 29) = 1;
obs(29, 36:55) = 1;

% Euclidean distance to nearest obstacle cell (in cells)
d_cells = bwdist(obs > 0);
d = d_cells * resolution;    % meters

lethal = 254;
inscribed = 253;

costmap = zeros(h, w);

% lethal cells
costmap(d <= 0) = lethal;

% inside inscribed radius
mask_in = (d > 0) & (d <= r_inscribed);
costmap(mask_in) = inscribed;

% inflation band
mask_inf = (d > r_inscribed) & (d <= r_inflation);
c = (inscribed - 1) .* exp(-cost_scaling .* (d(mask_inf) - r_inscribed)) + 1;
c = round(c);
c = max(1, min(inscribed-1, c));
costmap(mask_inf) = c;

% display
figure;
imagesc(costmap);
axis image; colorbar;
title('Inflated Costmap (0..254)');
