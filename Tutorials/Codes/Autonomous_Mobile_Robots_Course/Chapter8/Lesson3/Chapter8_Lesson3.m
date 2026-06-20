
% Chapter8_Lesson3.m
%
% Autonomous Mobile Robots (Control Engineering) - Chapter 8, Lesson 3
% Sensor Likelihoods for LiDAR and Vision
%
% This script demonstrates:
% 1) LiDAR likelihood-field model using a precomputed distance transform.
% 2) Bearing-only vision landmark likelihood with an outlier mixture.
% 3) Numerically stable normalization of particle weights from log-likelihoods.
%
% Robotics toolbox notes:
% - MATLAB Robotics System Toolbox: occupancyMap / binaryOccupancyMap, rayIntersection, ROS/ROS2 I/O.
% - Navigation Toolbox provides localization/planning utilities.
% - bwdist (Image Processing Toolbox) gives an exact Euclidean distance transform on grids.
%
% To run:
%   Chapter8_Lesson3

clear; clc;

%% Map (binary occupancy)
W = 120; H = 120; res = 0.05;  % cells, meters/cell
origin = [-3.0, -3.0];         % world of cell(1,1) corner
occ = zeros(H, W, 'uint8');
occ(61, 11:110) = 1;           % horizontal wall (MATLAB indexing)
occ(21:30, 86:95) = 1;         % pillar block

dist_field = compute_distance_field(occ, res);  % meters

%% LiDAR likelihood-field parameters
z_max = 8.0;
sigma_hit = 0.15;
w_hit = 0.95;
w_rand = 0.05;
max_dist = 2.0;

%% Vision landmark map (bearing-only)
landmarks = [ ...
    1, -1.0,  0.0; ...
    2,  1.0,  0.5; ...
    3,  0.0, -1.0  ...
]; % [id, x, y]
sigma_bearing = deg2rad(3.0);
eps_outlier = 0.10;

%% Particles
particles = [ ...
   -0.5, -0.8, deg2rad(90.0); ...
   -0.2, -0.6, deg2rad(88.0); ...
    0.8,  0.1, deg2rad(30.0)  ...
];

%% Synthetic measurements
rel_angles = linspace(deg2rad(-40), deg2rad(40), 9);
ranges = [2.8, 3.0, 3.2, 3.1, 3.0, 3.2, 2.9, 2.7, 2.6];

obs_ids = [1, 2, 3];
obs_bear = deg2rad([70.0, 20.0, -95.0]);

%% Compute log-weights
logw = zeros(size(particles,1), 1);
for i = 1:size(particles,1)
    pose = particles(i,:);
    ll_lidar = lidar_loglikelihood_field(pose, ranges, rel_angles, dist_field, res, origin, ...
        z_max, sigma_hit, w_hit, w_rand, max_dist);
    ll_vis = vision_bearing_loglikelihood(pose, obs_ids, obs_bear, landmarks, sigma_bearing, eps_outlier);
    logw(i) = ll_lidar + ll_vis;
end

w = normalize_log_weights(logw);
disp('log-weights:'); disp(logw.');
disp('weights:'); disp(w.');

%% Optional: create a minimal Simulink model (programmatically)
% The model demonstrates a MATLAB Function block computing a scan log-likelihood.
% Uncomment to generate:
% create_simulink_demo_model();

%% -------------------- Local functions --------------------
function dist_field = compute_distance_field(occ, res)
% Compute distance to nearest obstacle cell (in meters).
% If bwdist exists, use it; else approximate with a brushfire transform.
    if exist('bwdist', 'file') == 2
        dist_cells = bwdist(occ > 0);      % distance to nearest nonzero (obstacle)
        dist_field = dist_cells * res;
    else
        dist_field = brushfire_distance(occ, res);
    end
end

function dist = brushfire_distance(occ, res)
% Approximate Euclidean distance using multi-source Dijkstra on 8-neighbors.
    [H,W] = size(occ);
    INF = 1e9;
    dist = INF * ones(H,W);
    % priority queue emulation (inefficient but dependency-free)
    Q = [];
    [ys,xs] = find(occ > 0);
    for k = 1:numel(xs)
        dist(ys(k), xs(k)) = 0;
        Q(end+1,:) = [0, ys(k), xs(k)]; %#ok<AGROW>
    end
    nbh = [ -1 0 1.0; 1 0 1.0; 0 -1 1.0; 0 1 1.0; ...
            -1 -1 sqrt(2); -1 1 sqrt(2); 1 -1 sqrt(2); 1 1 sqrt(2) ];

    while ~isempty(Q)
        % extract min
        [~,ii] = min(Q(:,1));
        cur = Q(ii,:); Q(ii,:) = [];
        d = cur(1); y = cur(2); x = cur(3);
        if d > dist(y,x), continue; end
        for j = 1:size(nbh,1)
            yy = y + nbh(j,1); xx = x + nbh(j,2); c = nbh(j,3);
            if yy < 1 || yy > H || xx < 1 || xx > W, continue; end
            nd = d + c;
            if nd < dist(yy,xx)
                dist(yy,xx) = nd;
                Q(end+1,:) = [nd, yy, xx]; %#ok<AGROW>
            end
        end
    end
    dist = dist * res;
end

function ll = lidar_loglikelihood_field(pose, ranges, rel_angles, dist_field, res, origin, ...
    z_max, sigma_hit, w_hit, w_rand, max_dist)
% LiDAR likelihood field model (robust):
%   p = w_hit*exp(-d^2/(2*sigma^2)) + w_rand*(1/z_max)
%   ll = sum log(p)
    x = pose(1); y = pose(2); th = pose(3);
    sig2 = sigma_hit^2;
    inv_z = 1.0 / z_max;
    ll = 0.0;

    for k = 1:numel(ranges)
        z = min(max(ranges(k), 0.0), z_max);
        a = th + rel_angles(k);
        wx = x + z*cos(a); wy = y + z*sin(a);

        [gx,gy,inside] = world_to_grid(wx, wy, res, origin, size(dist_field));
        if inside
            d = min(max_dist, dist_field(gy,gx));
        else
            d = max_dist;
        end

        p_hit = exp(-(d*d)/(2.0*sig2));
        p = w_hit*p_hit + w_rand*inv_z;
        ll = ll + log(max(p, 1e-12));
    end
end

function ll = vision_bearing_loglikelihood(pose, ids, bearings, landmarks, sigma_bearing, eps_outlier)
% Bearing-only landmark likelihood with outlier mixture.
    x = pose(1); y = pose(2); th = pose(3);
    sig2 = sigma_bearing^2;
    normc = 1.0 / sqrt(2*pi*sig2);
    uni = 1.0 / (2*pi);
    ll = 0.0;

    for i = 1:numel(ids)
        id = ids(i);
        idx = find(landmarks(:,1) == id, 1);
        if isempty(idx)
            ll = ll + log(uni);
            continue;
        end
        lx = landmarks(idx,2); ly = landmarks(idx,3);
        pred = wrap_to_pi(atan2(ly - y, lx - x) - th);
        innov = wrap_to_pi(bearings(i) - pred);

        p_in = normc * exp(-(innov*innov)/(2.0*sig2));
        p = (1-eps_outlier)*p_in + eps_outlier*uni;
        ll = ll + log(max(p, 1e-15));
    end
end

function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function [gx,gy,inside] = world_to_grid(wx, wy, res, origin, sz)
    gx = floor((wx - origin(1))/res) + 1;
    gy = floor((wy - origin(2))/res) + 1;
    inside = (gx >= 1 && gx <= sz(2) && gy >= 1 && gy <= sz(1));
end

function w = normalize_log_weights(logw)
% Stable normalization via log-sum-exp.
    m = max(logw);
    s = sum(exp(logw - m));
    lse = m + log(s);
    w = exp(logw - lse);
    w = w / sum(w);
end

function create_simulink_demo_model()
% Programmatically create a minimal Simulink model with a MATLAB Function block.
% Requires Simulink.
    mdl = 'Chapter8_Lesson3_Simulink';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/pose'], 'Value', '[0;0;0]');
    add_block('simulink/Sources/Constant', [mdl '/ranges'], 'Value', '[1;1;1]');
    add_block('simulink/Sources/Constant', [mdl '/angles'], 'Value', '[-0.5;0;0.5]');
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/loglikelihood']);

    set_param([mdl '/loglikelihood'], 'Script', ...
        [ 'function ll = f(pose,ranges,angles)\n' ...
          '% Replace with a call to lidar_loglikelihood_field using base-workspace dist_field\n' ...
          'll = 0;\n' ...
          'end\n' ]);

    add_block('simulink/Sinks/Display', [mdl '/Display']);

    add_line(mdl, 'pose/1', 'loglikelihood/1');
    add_line(mdl, 'ranges/1', 'loglikelihood/2');
    add_line(mdl, 'angles/1', 'loglikelihood/3');
    add_line(mdl, 'loglikelihood/1', 'Display/1');

    save_system(mdl);
end
      