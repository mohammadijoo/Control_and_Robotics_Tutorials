% Chapter18_Lesson2.m
% Traversability + terrain classification (toy implementation) + Simulink model stub.
%
% Usage (MATLAB):
%   % Input CSV columns: x,y,z
%   Chapter18_Lesson2('points.csv', 0.25, 'traversability_map.csv');
%
% Or generate a synthetic point cloud:
%   Chapter18_Lesson2('', 0.25, 'traversability_map.csv');
%
% This is an educational script (no ROS required). For real robots, you can
% integrate with ROS Toolbox (ros2subscriber / ros2publisher) and Nav2-style
% costmap layers.

function Chapter18_Lesson2(points_csv, resolution, out_csv)

if nargin < 1, points_csv = ''; end
if nargin < 2, resolution = 0.25; end
if nargin < 3, out_csv = 'traversability_map.csv'; end

if ~isempty(points_csv)
    P = readmatrix(points_csv);
    P = P(:,1:3);
else
    P = synthetic_point_cloud(200000, 0);
end

x_min = min(P(:,1)); x_max = max(P(:,1));
y_min = min(P(:,2)); y_max = max(P(:,2));

Nx = ceil((x_max - x_min) / resolution);
Ny = ceil((y_max - y_min) / resolution);

H_sum = zeros(Ny, Nx);
C = zeros(Ny, Nx);

ix = floor((P(:,1) - x_min) / resolution) + 1;
iy = floor((P(:,2) - y_min) / resolution) + 1;

valid = ix >= 1 & ix <= Nx & iy >= 1 & iy <= Ny;
ix = ix(valid); iy = iy(valid); z = P(valid,3);

for i = 1:numel(z)
    H_sum(iy(i), ix(i)) = H_sum(iy(i), ix(i)) + z(i);
    C(iy(i), ix(i)) = C(iy(i), ix(i)) + 1;
end

H = nan(Ny, Nx);
mask = C > 0;
H(mask) = H_sum(mask) ./ C(mask);

Hf = fillmissing(H, 'nearest');

% Gradient-based slope
[dhdy, dhdx] = gradient(Hf, resolution, resolution);
slope_rad = atan(sqrt(dhdx.^2 + dhdy.^2));
slope_deg = slope_rad * 180/pi;

% Roughness: local std in 3x3 window
rough = local_std(Hf, 1);

% Step height: local max-min in 3x3
step = local_step(Hf, 1);

% Simple calibrated model (illustrative weights; learn for your robot)
b  = 3.0;
w1 = -6.0;   % slope_rad
w2 = -40.0;  % rough [m]
w3 = -25.0;  % step [m]
p_trav = 1 ./ (1 + exp(-(b + w1*slope_rad + w2*rough + w3*step)));
p_trav(~mask) = NaN;

% Export per-cell CSV
fid = fopen(out_csv, 'w');
fprintf(fid, 'x,y,p_trav,slope_deg,rough_m,step_m,count\n');
for iy0 = 1:Ny
    yc = y_min + (iy0 - 0.5) * resolution;
    for ix0 = 1:Nx
        if ~mask(iy0, ix0), continue; end
        xc = x_min + (ix0 - 0.5) * resolution;
        fprintf(fid, '%.6f,%.6f,%.8f,%.4f,%.6f,%.6f,%d\n', ...
            xc, yc, p_trav(iy0,ix0), slope_deg(iy0,ix0), rough(iy0,ix0), step(iy0,ix0), C(iy0,ix0));
    end
end
fclose(fid);

disp(['Exported: ', out_csv]);

% Optional: create a Simulink model that implements the logistic traversability block
% (requires Simulink). Uncomment to generate:
% create_traversability_simulink_model();

end

function P = synthetic_point_cloud(n, seed)
rng(seed);
x = -10 + 20*rand(n,1);
y = -10 + 20*rand(n,1);

z = 0.05*x + 0.02*y;
% bumps
bumps = [-3 2 0.35 1.2; 4 -1.5 0.25 0.9; 1 5 0.30 1.0];
for i=1:size(bumps,1)
    cx=bumps(i,1); cy=bumps(i,2); a=bumps(i,3); s=bumps(i,4);
    z = z + a * exp(-((x-cx).^2 + (y-cy).^2)/(2*s^2));
end
% ditch
ditch = x > -2 & x < 2 & y > -6 & y < -3;
z(ditch) = z(ditch) - 0.6;

z = z + 0.02*randn(n,1);
P = [x y z];
end

function S = local_std(A, k)
[Ny, Nx] = size(A);
S = zeros(Ny, Nx);
for y=1:Ny
    ys = max(1,y-k):min(Ny,y+k);
    for x=1:Nx
        xs = max(1,x-k):min(Nx,x+k);
        patch = A(ys, xs);
        S(y,x) = std(patch(:), 0);
    end
end
end

function D = local_step(A, k)
[Ny, Nx] = size(A);
D = zeros(Ny, Nx);
for y=1:Ny
    ys = max(1,y-k):min(Ny,y+k);
    for x=1:Nx
        xs = max(1,x-k):min(Nx,x+k);
        patch = A(ys, xs);
        D(y,x) = max(patch(:)) - min(patch(:));
    end
end
end

function create_traversability_simulink_model()
% Programmatically create a small Simulink model:
% Inputs: slope_rad, rough, step
% Output: p_trav = sigmoid(b + w1*slope + w2*rough + w3*step)
%
% This is intended as a teaching aid: you can connect this block to a
% perception pipeline and feed signals from ROS Toolbox or logged data.

model = 'Chapter18_Lesson2_Simulink';
new_system(model);
open_system(model);

add_block('simulink/Sources/In1', [model '/slope_rad'], 'Position', [30 30 60 50]);
add_block('simulink/Sources/In1', [model '/rough'],     'Position', [30 80 60 100]);
add_block('simulink/Sources/In1', [model '/step'],      'Position', [30 130 60 150]);

add_block('simulink/User-Defined Functions/MATLAB Function', [model '/Traversability'], ...
    'Position', [160 55 320 145]);
set_param([model '/Traversability'], 'Script', ...
    sprintf(['function p = f(slope_rad, rough, step)\n' ...
             '%% logistic traversability\n' ...
             'b=3.0; w1=-6.0; w2=-40.0; w3=-25.0;\n' ...
             's = b + w1*slope_rad + w2*rough + w3*step;\n' ...
             'p = 1./(1+exp(-s));\n' ...
             'end\n']));

add_block('simulink/Sinks/Out1', [model '/p_trav'], 'Position', [380 85 410 105]);

add_line(model, 'slope_rad/1', 'Traversability/1');
add_line(model, 'rough/1',     'Traversability/2');
add_line(model, 'step/1',      'Traversability/3');
add_line(model, 'Traversability/1', 'p_trav/1');

save_system(model);
disp(['Created Simulink model: ', model]);

end
