% Chapter 9 - Mapping Representations for Mobile Robots
% Lesson 1 - Occupancy Grid Mapping
%
% This script implements a simple 2D occupancy grid mapper using a log-odds grid.
% It also demonstrates a comparable workflow using Robotics System Toolbox (if available).
%
% Run:
%   Chapter9_Lesson1
%
% Notes:
% - For the from-scratch section, no toolboxes are required.
% - For the toolbox section, you need Robotics System Toolbox.

function Chapter9_Lesson1()
  close all; clc;

  % Map parameters
  W = 120; H = 100;
  res = 0.05;                % meters per cell
  origin = [0.0, 0.0];
  prior = 0.5;
  p_occ  = 0.75;
  p_free = 0.35;
  Lmin = -10; Lmax = 10;

  L0 = logit(prior);
  L  = L0 * ones(H, W);
  l_occ  = logit(p_occ)  - logit(prior);
  l_free = logit(p_free) - logit(prior);

  % Ground truth (only for generating synthetic measurements)
  gt = buildToyWorld(W, H);

  % Trajectory
  traj = zeros(25, 3);
  for k = 1:25
    traj(k,:) = [0.8 + 0.06*(k-1), 0.9 + 0.01*(k-1), 0.15];
  end

  % Sensor
  fov = pi;          % 180 deg
  nRays = 121;
  angles = linspace(-fov/2, fov/2, nRays);
  zMax = 3.0;

  % Mapping loop
  for k = 1:size(traj,1)
    pose = traj(k,:);
    for i = 1:numel(angles)
      ang = angles(i);
      z = castRayGrid(gt, origin, res, W, H, pose, ang, zMax);
      [L] = updateRay(L, origin, res, W, H, pose, ang, z, zMax, l_free, l_occ, Lmin, Lmax);
    end
  end

  % Visualize occupancy probabilities
  P = inv_logit(L);
  figure('Name','Occupancy probability (from scratch)');
  imagesc(P); axis image; set(gca, 'YDir','normal');
  title('Occupancy probability (after mapping)');
  xlabel('grid x'); ylabel('grid y'); colorbar;

  figure('Name','Ground truth (simulation only)');
  imagesc(gt); axis image; set(gca, 'YDir','normal');
  title('Ground truth occupancy (used only for synthetic rays)');
  xlabel('grid x'); ylabel('grid y');

  % Optional: Robotics System Toolbox demonstration
  demoRoboticsToolboxIfAvailable(gt, res);
end

function demoRoboticsToolboxIfAvailable(gt, res)
  if exist('occupancyMap','class') ~= 8
    fprintf('Robotics System Toolbox not detected; skipping toolbox demo.\n');
    return;
  end

  fprintf('Robotics System Toolbox detected; running a brief occupancyMap demo...\n');
  map = occupancyMap(size(gt,2)*res, size(gt,1)*res, 1/res); % width[m], height[m], cells/m
  setOccupancy(map, gt);  % interpret gt as occupancy (0/1)
  figure('Name','Toolbox: occupancyMap visualization');
  show(map); title('Toolbox map visualization');
end

function gt = buildToyWorld(W, H)
  gt = zeros(H, W, 'uint8');

  % Border walls
  gt(1,:)   = 1;
  gt(end,:) = 1;
  gt(:,1)   = 1;
  gt(:,end) = 1;

  % Rectangle obstacle
  gt(31:50, 56:75) = 1;

  % Small obstacle
  gt(71:80, 26:35) = 1;
end

function z = castRayGrid(gt, origin, res, W, H, pose, relAng, zMax)
  step = 0.5 * res;
  x = pose(1); y = pose(2); theta = pose(3);
  a = theta + relAng;

  dist = 0.0;
  while dist < zMax
    xt = x + dist*cos(a);
    yt = y + dist*sin(a);
    [gx, gy] = worldToGrid(origin, res, xt, yt);
    if gx >= 1 && gx <= W && gy >= 1 && gy <= H
      if gt(gy, gx) == 1
        z = dist; return;
      end
    else
      z = dist; return;
    end
    dist = dist + step;
  end
  z = zMax;
end

function [L] = updateRay(L, origin, res, W, H, pose, relAng, z, zMax, l_free, l_occ, Lmin, Lmax)
  x = pose(1); y = pose(2); theta = pose(3);
  z = max(0.0, min(z, zMax));
  a = theta + relAng;

  xe = x + z*cos(a);
  ye = y + z*sin(a);

  [x0, y0] = worldToGrid(origin, res, x, y);
  [x1, y1] = worldToGrid(origin, res, xe, ye);

  if x0 < 1 || x0 > W || y0 < 1 || y0 > H
    return;
  end

  cells = bresenham(x0, y0, x1, y1);
  if isempty(cells), return; end

  hit = (z < zMax - 1e-6);

  if hit
    freeCells = cells(1:end-1,:);
  else
    freeCells = cells;
  end

  for i = 1:size(freeCells,1)
    cx = freeCells(i,1); cy = freeCells(i,2);
    if cx >= 1 && cx <= W && cy >= 1 && cy <= H
      L(cy, cx) = min(Lmax, max(Lmin, L(cy, cx) + l_free));
    end
  end

  if hit
    cx = cells(end,1); cy = cells(end,2);
    if cx >= 1 && cx <= W && cy >= 1 && cy <= H
      L(cy, cx) = min(Lmax, max(Lmin, L(cy, cx) + l_occ));
    end
  end
end

function [gx, gy] = worldToGrid(origin, res, x, y)
  gx = floor((x - origin(1)) / res) + 1; % MATLAB 1-based
  gy = floor((y - origin(2)) / res) + 1;
end

function pts = bresenham(x0, y0, x1, y1)
  dx = abs(x1 - x0);
  dy = abs(y1 - y0);
  sx = 1; if x1 < x0, sx = -1; end
  sy = 1; if y1 < y0, sy = -1; end
  err = dx - dy;

  x = x0; y = y0;
  pts = zeros(0,2);
  while true
    pts(end+1,:) = [x, y]; %#ok<AGROW>
    if x == x1 && y == y1
      break;
    end
    e2 = 2*err;
    if e2 > -dy
      err = err - dy;
      x = x + sx;
    end
    if e2 < dx
      err = err + dx;
      y = y + sy;
    end
  end
end

function y = logit(p)
  eps = 1e-12;
  p = min(1.0-eps, max(eps, p));
  y = log(p/(1-p));
end

function p = inv_logit(L)
  p = 1 ./ (1 + exp(-L));
end
