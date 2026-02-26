% Chapter 1 — Lesson 3: Environment Representations for AMR
% File: Chapter1_Lesson3.m
%
% This script demonstrates:
%   1) Raster occupancy grid using rectangle obstacles
%   2) Signed distance field via bwdist
%   3) Disc-robot inflation in configuration space
%   4) Programmatic creation of a small Simulink model skeleton
%
% Requirements:
%   - Image Processing Toolbox for bwdist

clear; clc;

% Grid
H = 160; W = 200;
res = 0.05; % meters per cell

occ = zeros(H, W, 'uint8');

% Rectangle obstacles [r0 c0 r1 c1] inclusive
rects = [
    30  40  55 100
    90 120 130 180
   110  20 140  45
];

for k = 1:size(rects,1)
    r0 = max(1, rects(k,1)); c0 = max(1, rects(k,2));
    r1 = min(H, rects(k,3)); c1 = min(W, rects(k,4));
    occ(r0:r1, c0:c1) = 1;
end

occ_bool = occ == 1;
free_bool = ~occ_bool;

% Distance to obstacles for free cells
d_free_to_obs = bwdist(occ_bool) * res;

% Distance to free for obstacle cells
d_obs_to_free = bwdist(free_bool) * res;

% Signed distance field (SDF): positive outside, negative inside
sdf = double(d_free_to_obs);
sdf(occ_bool) = -double(d_obs_to_free(occ_bool));

% Disc-robot inflation
robot_radius = 0.30;
occ_infl = uint8(sdf < robot_radius);

% Export CSV
writematrix(occ, 'occupancy_grid_matlab.csv');
writematrix(occ_infl, 'inflated_occupancy_grid_matlab.csv');
writematrix(sdf, 'signed_distance_field_matlab.csv');

fprintf('Grid: H=%d, W=%d, res=%.3f m\n', H, W, res);
fprintf('Free ratio (original): %.4f\n', mean(occ(:)==0));
fprintf('Free ratio (inflated): %.4f\n', mean(occ_infl(:)==0));
fprintf('Wrote: occupancy_grid_matlab.csv, inflated_occupancy_grid_matlab.csv, signed_distance_field_matlab.csv\n');

% Optional visualization (comment out if you do not want figures)
figure; imagesc(occ); axis image; title('Occupancy grid (1=occupied)'); colorbar;
figure; imagesc(sdf); axis image; title('Signed distance field (meters)'); colorbar;

% -------------------------------------------------------------------------
% Simulink skeleton: create a model with a Constant block feeding a MATLAB
% Function block that computes a distance transform. This is a minimal
% starting point, not a complete navigation pipeline.
% -------------------------------------------------------------------------
try
    mdl = 'Chapter1_Lesson3_SDF_Model';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/OccupancyGrid'], ...
        'Value', 'occ', 'Position', [50 60 200 110]);

    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/ComputeSDF'], ...
        'Position', [260 50 430 130]);

    add_block('simulink/Sinks/To Workspace', [mdl '/SDF_to_WS'], ...
        'VariableName', 'sdf_sim', 'Position', [500 60 650 110]);

    add_line(mdl, 'OccupancyGrid/1', 'ComputeSDF/1');
    add_line(mdl, 'ComputeSDF/1', 'SDF_to_WS/1');

    % Set MATLAB Function block code
    fcnPath = [mdl '/ComputeSDF'];
    set_param(fcnPath, 'Script', sprintf([ ...
        'function sdf_out = fcn(occ_in)\n' ...
        '%% occ_in: uint8 matrix, 1=occupied\n' ...
        'occ_bool = occ_in == 1;\n' ...
        'free_bool = ~occ_bool;\n' ...
        'd_free_to_obs = bwdist(occ_bool);\n' ...
        'd_obs_to_free = bwdist(free_bool);\n' ...
        'sdf_out = double(d_free_to_obs);\n' ...
        'sdf_out(occ_bool) = -double(d_obs_to_free(occ_bool));\n' ...
        'end\n' ...
    ]));

    save_system(mdl);
    fprintf('Created Simulink model: %s.slx\n', mdl);
catch ME
    fprintf('Simulink model creation skipped (reason: %s)\n', ME.message);
end
