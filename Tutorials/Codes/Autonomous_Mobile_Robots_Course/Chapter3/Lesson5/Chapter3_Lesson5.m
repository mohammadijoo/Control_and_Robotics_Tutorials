% Chapter3_Lesson5.m
% Autonomous Mobile Robots (Control Engineering major)
% Chapter 3 — Nonholonomic Motion and Feasibility for AMR
% Lesson 5 — Feasibility Checks for Candidate Paths
%
% This MATLAB script implements feasibility checks for a candidate path:
%  1) curvature/steering bound
%  2) steering rate bound (via dkappa/ds and time scaling)
%  3) speed and acceleration bounds (forward-backward pass)
%  4) collision clearance on occupancy grid (disk approximation) via bwdist
%
% Toolboxes that help (optional):
%  - Robotics System Toolbox / Navigation Toolbox (maps, planners)
%  - Image Processing Toolbox (bwdist distance transform)
%
% The code is written to be readable and course-aligned; students can refactor
% into functions/classes as an exercise.

clear; clc;

%% --- Limits (example) ---
lim.wheelbase_m          = 0.33;
lim.delta_max_rad        = 0.45;
lim.delta_dot_max_rad_s  = 0.75;
lim.v_max_m_s            = 1.2;
lim.a_long_max_m_s2      = 0.8;
lim.a_lat_max_m_s2       = 1.5;
lim.robot_radius_m       = 0.25;
lim.clearance_margin_m   = 0.05;

%% --- Candidate path (example: circular arc) ---
N = 200;
t = linspace(0,1,N)';
R = 4.0;
ang = 0.6*t;
xy = [R*sin(ang), R*(1-cos(ang))];

%% --- Occupancy grid (example) ---
res = 0.05;       % meters per cell
W = round(10.0/res);
H = round(10.0/res);
occ = zeros(H,W); % 1 obstacle, 0 free

% obstacle patch near (2,1)
ox = round(2.0/res);
oy = round(1.0/res);
occ(max(1,oy-3):min(H,oy+3), max(1,ox-3):min(W,ox+3)) = 1;

origin = [0,0];   % (x0,y0) world coordinates of cell (1,1)

%% --- Feasibility check ---
[ok, rep] = check_path_feasibility(xy, occ, res, origin, lim);
disp(ok);
disp(rep);

%% --- Simulink (optional): create a minimal model programmatically ---
% This section demonstrates how you could wrap feasibility checking into a
% MATLAB Function block inside Simulink. It is optional and may require
% Simulink installed and licensed.

% Uncomment to generate the model:
% create_simulink_feasibility_model();

%% ======= Functions =======

function [ok, rep] = check_path_feasibility(xy, occ, res, origin, lim)
  if size(xy,2) ~= 2 || size(xy,1) < 3
    ok = false; rep.reason = 'invalid_path_shape'; return;
  end

  s = arc_length(xy);
  kappa = curvature_discrete(xy);
  dkds = dkappa_ds(kappa, s);

  kappa_max = tan(lim.delta_max_rad)/lim.wheelbase_m;
  if max(abs(kappa)) > kappa_max + 1e-9
    ok = false;
    rep.reason = 'curvature_limit_violation';
    rep.kappa_max = kappa_max;
    rep.kappa_abs_max = max(abs(kappa));
    return;
  end

  [v, dt] = time_scaling(s, kappa, lim);

  delta = atan(lim.wheelbase_m * kappa);
  kappa_dot = v .* dkds;
  delta_dot = lim.wheelbase_m .* (cos(delta).^2) .* kappa_dot;

  if max(abs(delta_dot)) > lim.delta_dot_max_rad_s + 1e-9
    ok = false;
    rep.reason = 'steering_rate_violation';
    rep.delta_dot_abs_max = max(abs(delta_dot));
    rep.delta_dot_max = lim.delta_dot_max_rad_s;
    return;
  end

  if ~isempty(occ)
    [okcol, colrep] = collision_check_disk(xy, occ, res, origin, lim);
    if ~okcol
      ok = false;
      rep.reason = 'collision_violation';
      rep.collision = colrep;
      return;
    end
  else
    colrep.skipped = true;
  end

  ok = true;
  rep.reason = 'ok';
  rep.path_length_m = s(end);
  rep.kappa_abs_max = max(abs(kappa));
  rep.kappa_max = kappa_max;
  rep.v_min = min(v);
  rep.v_max = max(v);
  rep.delta_dot_abs_max = max(abs(delta_dot));
  rep.total_time_s = sum(dt);
  rep.collision = colrep;
end

function s = arc_length(xy)
  d = sqrt(sum(diff(xy,1,1).^2,2));
  s = [0; cumsum(d)];
end

function kappa = curvature_discrete(xy)
  n = size(xy,1);
  kappa = zeros(n,1);
  epsv = 1e-12;
  for i = 2:n-1
    p0 = xy(i-1,:); p1 = xy(i,:); p2 = xy(i+1,:);
    a = p1 - p0; c = p2 - p0; b = p2 - p1;
    la = norm(a); lb = norm(b); lc = norm(c);
    area2 = a(1)*c(2) - a(2)*c(1);
    denom = la*lb*lc + epsv;
    kappa(i) = 2*area2/denom;
  end
  kappa(1) = kappa(2);
  kappa(end) = kappa(end-1);
end

function dk = dkappa_ds(kappa, s)
  n = numel(kappa);
  dk = zeros(n,1);
  epsv = 1e-12;
  for i = 2:n-1
    dk(i) = (kappa(i+1)-kappa(i-1))/((s(i+1)-s(i-1))+epsv);
  end
  dk(1) = (kappa(2)-kappa(1))/((s(2)-s(1))+epsv);
  dk(end) = (kappa(end)-kappa(end-1))/((s(end)-s(end-1))+epsv);
end

function [v, dt] = time_scaling(s, kappa, lim)
  n = numel(s);
  ds = diff(s);
  vcap = lim.v_max_m_s * ones(n,1);

  for i = 1:n
    if abs(kappa(i)) > 1e-9
      vlat = sqrt(max(lim.a_lat_max_m_s2/abs(kappa(i)), 0));
      vcap(i) = min(vcap(i), vlat);
    end
  end

  w = min(vcap, lim.v_max_m_s).^2;

  % forward pass
  for i = 1:n-1
    w(i+1) = min(w(i+1), w(i) + 2*lim.a_long_max_m_s2*max(ds(i),0));
  end
  % backward pass
  for i = n-1:-1:1
    w(i) = min(w(i), w(i+1) + 2*lim.a_long_max_m_s2*max(ds(i),0));
  end

  v = sqrt(max(w,0));
  epsv = 1e-12;
  dt = zeros(n-1,1);
  for i = 1:n-1
    vavg = 0.5*(v(i)+v(i+1));
    dt(i) = ds(i)/(vavg+epsv);
  end
end

function [ok, rep] = collision_check_disk(xy, occ, res, origin, lim)
  needed = lim.robot_radius_m + lim.clearance_margin_m;

  % Distance to nearest obstacle: compute on "free" cells.
  % bwdist computes distance to nearest nonzero; so use occ==1 as "object".
  if exist('bwdist','file')
    dist_cells = bwdist(occ==1);
    dist_m = dist_cells * res;
  else
    % Fallback: chamfer distance using iterative relaxation
    dist_m = chamfer_distance(occ, res);
  end

  H = size(occ,1); W = size(occ,2);
  min_clear = inf; worst = 1;

  for i = 1:size(xy,1)
    gx = floor((xy(i,1)-origin(1))/res) + 1;
    gy = floor((xy(i,2)-origin(2))/res) + 1;
    if gx < 1 || gx > W || gy < 1 || gy > H
      ok = false; rep.reason='path_outside_grid'; rep.index=i; return;
    end
    c = dist_m(gy,gx);
    if c < min_clear
      min_clear = c; worst = i;
    end
    if c < needed
      ok = false; rep.reason='insufficient_clearance'; rep.index=i;
      rep.clearance_m = c; rep.needed_m = needed; return;
    end
  end

  ok = true;
  rep.min_clearance_m = min_clear;
  rep.worst_index = worst;
  rep.needed_m = needed;
end

function dist = chamfer_distance(occ, res)
  % Very simple chamfer-like distance (not exact Euclidean). Obstacles dist=0.
  [H,W] = size(occ);
  INF = 1e9;
  dist = INF*ones(H,W);
  dist(occ==1) = 0;

  % forward
  for y=2:H
    for x=2:W-1
      dist(y,x) = min([dist(y,x), dist(y-1,x)+1, dist(y,x-1)+1, dist(y-1,x-1)+sqrt(2), dist(y-1,x+1)+sqrt(2)]);
    end
  end
  % backward
  for y=H-1:-1:1
    for x=W-1:-1:2
      dist(y,x) = min([dist(y,x), dist(y+1,x)+1, dist(y,x+1)+1, dist(y+1,x+1)+sqrt(2), dist(y+1,x-1)+sqrt(2)]);
    end
  end
  dist = dist * res;
end

function create_simulink_feasibility_model()
  model = 'Chapter3_Lesson5_FeasibilityModel';
  if bdIsLoaded(model), close_system(model,0); end
  new_system(model); open_system(model);

  add_block('simulink/Sources/Constant', [model '/xy'], 'Value', 'xy', 'Position', [30 50 130 90]);
  add_block('simulink/Sources/Constant', [model '/occ'], 'Value', 'occ', 'Position', [30 120 130 160]);
  add_block('simulink/User-Defined Functions/MATLAB Function', [model '/FeasibilityCheck'], 'Position', [220 70 420 160]);
  add_block('simulink/Sinks/Display', [model '/Display'], 'Position', [520 95 620 135]);

  % MATLAB Function block code
  code = [
    "function ok = FeasibilityCheck(xy_in, occ_in)\n" + ...
    "%#codegen\n" + ...
    "% Minimal wrapper around check_path_feasibility (workspace function).\n" + ...
    "[ok, ~] = check_path_feasibility(xy_in, occ_in, " + num2str(0.05) + ", [0,0], struct(...\n" + ...
    " 'wheelbase_m',0.33,'delta_max_rad',0.45,'delta_dot_max_rad_s',0.75,'v_max_m_s',1.2,\n" + ...
    " 'a_long_max_m_s2',0.8,'a_lat_max_m_s2',1.5,'robot_radius_m',0.25,'clearance_margin_m',0.05));\n" + ...
    "end\n"
  ];
  set_param([model '/FeasibilityCheck'], 'Script', code);

  add_line(model, 'xy/1', 'FeasibilityCheck/1');
  add_line(model, 'occ/1', 'FeasibilityCheck/2');
  add_line(model, 'FeasibilityCheck/1', 'Display/1');

  save_system(model);
  fprintf('Created Simulink model: %s.slx\n', model);
end
