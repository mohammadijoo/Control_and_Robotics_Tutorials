% Chapter20_Lesson1.m
% Chapter 20 - Lesson 1
% Problem Definition and Environment Setup (MATLAB / Simulink helper)
%
% This script:
% 1) Creates an occupancy/traversability environment
% 2) Defines a mission and robot/sensor parameters
% 3) Checks battery/time feasibility
% 4) Optionally builds a minimal Simulink model programmatically

clear; clc; close all;
rng(42);

%% Mission and platform specification
mission.name = 'Capstone_AMR_IndoorOutdoor_Mix';
mission.map_resolution_m = 0.2;
mission.max_time_s = 1200;
mission.min_success_prob = 0.90;
mission.max_collisions = 0;

robot.radius_m = 0.25;
robot.v_max_mps = 1.2;
robot.w_max_radps = 1.8;
robot.battery_wh = 180;
robot.avg_power_w = 85;

sensors.lidar_range_m = 15;
sensors.lidar_fov_deg = 270;
sensors.lidar_rate_hz = 10;
sensors.imu_rate_hz = 100;
sensors.wheel_rate_hz = 50;
sensors.gps_rate_hz = 5;
sensors.gps_available = true;

env.width_m = 60;
env.height_m = 40;
env.obstacle_density = 0.14;
env.slip_regions = 4;

%% Environment generation
nx = ceil(env.width_m / mission.map_resolution_m);
ny = ceil(env.height_m / mission.map_resolution_m);

occ = rand(ny, nx) < env.obstacle_density;
trav = 1.0 + 0.5 * rand(ny, nx);

margin = max(2, ceil(0.8 / mission.map_resolution_m));
occ(1:margin, :) = 0; occ(end-margin+1:end, :) = 0;
occ(:, 1:margin) = 0; occ(:, end-margin+1:end) = 0;

for k = 1:env.slip_regions
    cx = randi(nx); cy = randi(ny);
    rad = max(4, floor(min(nx, ny) / 10));
    [X, Y] = meshgrid(1:nx, 1:ny);
    mask = (X - cx).^2 + (Y - cy).^2 <= rad^2;
    trav(mask) = trav(mask) + 1.0 + 0.8 * rand;
end
trav(occ == 1) = inf;

%% Start and goal placement
free_idx = find(~occ);
assert(numel(free_idx) >= 2, 'Not enough free cells.');
picked = false;
for it = 1:2000
    a = free_idx(randi(numel(free_idx)));
    b = free_idx(randi(numel(free_idx)));
    [sy, sx] = ind2sub([ny, nx], a);
    [gy, gx] = ind2sub([ny, nx], b);
    d = hypot((gx - sx) * mission.map_resolution_m, (gy - sy) * mission.map_resolution_m);
    if d >= 0.4 * max(nx, ny) * mission.map_resolution_m
        picked = true;
        break;
    end
end
if ~picked
    [sy, sx] = ind2sub([ny, nx], free_idx(1));
    [gy, gx] = ind2sub([ny, nx], free_idx(end));
end

mission.start_xytheta = [sx * mission.map_resolution_m, sy * mission.map_resolution_m, 0];
mission.goal_xy = [gx * mission.map_resolution_m, gy * mission.map_resolution_m];

%% Feasibility checks
available_time_s = (robot.battery_wh / robot.avg_power_w) * 3600;
time_margin_s = available_time_s - mission.max_time_s;

sensor_rates_ok = (sensors.wheel_rate_hz >= 20) && ...
                  (sensors.imu_rate_hz >= 50) && ...
                  (sensors.lidar_rate_hz >= 5);

fprintf('Mission: %s\n', mission.name);
fprintf('Start = [%.2f, %.2f, %.2f]\n', mission.start_xytheta);
fprintf('Goal  = [%.2f, %.2f]\n', mission.goal_xy);
fprintf('Battery available time [s] = %.1f\n', available_time_s);
fprintf('Time margin [s] = %.1f\n', time_margin_s);
fprintf('Sensor rates OK? %d\n', sensor_rates_ok);

%% Visualize
figure('Name', 'Chapter20_Lesson1 Environment');
subplot(1,2,1);
imagesc(occ); axis image; title('Occupancy Grid (1 = obstacle)');
hold on; plot(sx, sy, 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(gx, gy, 'rx', 'MarkerSize', 8, 'LineWidth', 2); hold off;

subplot(1,2,2);
imagesc(trav); axis image; title('Traversability Cost');
colorbar;

%% Save outputs
save('Chapter20_Lesson1_env.mat', 'mission', 'robot', 'sensors', 'env', 'occ', 'trav');

%% Optional Simulink skeleton (programmatic)
% This creates a simple simulation skeleton with reference and kinematic integrator.
modelName = 'Chapter20_Lesson1_Simulink';
if exist(modelName, 'file') == 4
    close_system(modelName, 0);
end
new_system(modelName); open_system(modelName);

add_block('simulink/Sources/Constant', [modelName '/v_ref'], 'Value', '0.5', ...
    'Position', [30 50 80 80]);
add_block('simulink/Sources/Constant', [modelName '/w_ref'], 'Value', '0.2', ...
    'Position', [30 120 80 150]);
add_block('simulink/Continuous/Integrator', [modelName '/x_int'], ...
    'Position', [220 40 250 70]);
add_block('simulink/Continuous/Integrator', [modelName '/y_int'], ...
    'Position', [220 110 250 140]);
add_block('simulink/Continuous/Integrator', [modelName '/th_int'], ...
    'Position', [220 180 250 210]);
add_block('simulink/Sinks/Scope', [modelName '/scope'], ...
    'Position', [380 80 430 140]);

% NOTE: For a full nonholonomic model, students will add trig blocks and couplings in later lessons.
save_system(modelName);
fprintf('Saved %s.slx (skeleton model)\n', modelName);
