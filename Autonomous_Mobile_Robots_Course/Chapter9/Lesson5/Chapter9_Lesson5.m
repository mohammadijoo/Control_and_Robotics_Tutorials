% Chapter 9 — Mapping Representations for Mobile Robots
% Lesson 5 (Lab): Build a 2D Occupancy Grid from LiDAR
%
% Self-contained MATLAB script:
% - Defines a world with circular obstacles + boundary walls
% - Simulates robot trajectory and 2D LiDAR ranges
% - Builds occupancy grid using inverse sensor model + log-odds updates
% - Visualizes occupancy probabilities
%
% Run:
%   Chapter9_Lesson5

function Chapter9_Lesson5()
  rng(7);

  % World (meters)
  world.xmin = -10; world.xmax = 10;
  world.ymin = -10; world.ymax = 10;
  circles = [
    -3.0,  2.0, 1.2;
     2.5, -1.0, 1.0;
     4.0,  4.0, 1.5;
    -4.5, -4.0, 1.0
  ];

  % LiDAR
  nBeams = 360;
  zMax = 8.0;
  angles = linspace(-pi, pi, nBeams+1); angles(end) = [];

  % Grid
  res = 0.1;
  origin = [-10, -10];
  widthM = 20; heightM = 20;
  w = ceil(widthM / res);
  h = ceil(heightM / res);

  p0 = 0.5;
  l0 = logit(p0);
  lMin = -8; lMax = 8;
  logOdds = l0 * ones(h, w);

  % Inverse sensor model params
  pOcc = 0.70; pFree = 0.30;
  lOcc = logit(pOcc); lFree = logit(pFree);
  alpha = 0.2;

  % Trajectory (known poses)
  T = 220;
  poses = zeros(T, 3);
  for t = 1:T
    ang = 2*pi*(t-1)/T;
    x = 6*cos(ang);
    y = 6*sin(ang);
    theta = ang + pi/2;
    poses(t, :) = [x, y, theta];
  end

  % Mapping
  for t = 1:T
    pose = poses(t, :);
    z = simulateLidar(world, circles, pose, angles, zMax, 0.02);
    for k = 1:nBeams
      logOdds = updateRay(logOdds, origin, res, pose, angles(k), z(k), zMax, l0, lOcc, lFree, lMin, lMax, alpha);
    end
  end

  % Visualization
  P = logistic(logOdds);
  figure('Color','w');
  imagesc([world.xmin world.xmax], [world.ymin world.ymax], P);
  axis xy equal tight;
  colormap(parula); colorbar;
  title('2D Occupancy Grid from Simulated LiDAR');
  xlabel('x [m]'); ylabel('y [m]');
  hold on;

  th = linspace(0, 2*pi, 200);
  for i = 1:size(circles,1)
    cx = circles(i,1); cy = circles(i,2); r = circles(i,3);
    plot(cx + r*cos(th), cy + r*sin(th), 'LineWidth', 1.5);
  end
  plot(poses(:,1), poses(:,2), 'LineWidth', 1.0);
  hold off;

  % Optional: save PGM-like image
  % imwrite(uint8(P*255), 'Chapter9_Lesson5_map_matlab.png');
end

function l = logit(p)
  p = min(max(p, 1e-9), 1 - 1e-9);
  l = log(p/(1-p));
end

function P = logistic(L)
  P = 1 ./ (1 + exp(-L));
end

function z = simulateLidar(world, circles, pose, angles, zMax, sigmaR)
  n = numel(angles);
  z = zMax * ones(n,1);
  px = pose(1); py = pose(2); th = pose(3);
  for k = 1:n
    ang = th + angles(k);
    dx = cos(ang); dy = sin(ang);

    hits = [];

    % boundary walls
    tb = rayAABB(px, py, dx, dy, world.xmin, world.xmax, world.ymin, world.ymax);
    if ~isnan(tb), hits(end+1) = tb; end

    % circles
    for i = 1:size(circles,1)
      c = circles(i,:);
      t = rayCircle(px, py, dx, dy, c(1), c(2), c(3));
      if ~isnan(t), hits(end+1) = t; end
    end

    if ~isempty(hits)
      r = min(hits);
      if r <= zMax
        z(k) = min(max(r + sigmaR*randn(), 0), zMax);
      end
    end
  end
end

function t = rayCircle(px, py, dx, dy, cx, cy, r)
  ox = px - cx; oy = py - cy;
  b = 2*(ox*dx + oy*dy);
  cterm = ox^2 + oy^2 - r^2;
  disc = b^2 - 4*cterm;
  if disc < 0
    t = NaN; return;
  end
  s = sqrt(disc);
  t1 = (-b - s)/2;
  t2 = (-b + s)/2;
  cand = [t1 t2];
  cand = cand(cand >= 0);
  if isempty(cand), t = NaN; else, t = min(cand); end
end

function t = rayAABB(px, py, dx, dy, xmin, xmax, ymin, ymax)
  tmin = -Inf; tmax = Inf;
  if abs(dx) < 1e-12
    if px < xmin || px > xmax, t = NaN; return; end
  else
    tx1 = (xmin - px)/dx; tx2 = (xmax - px)/dx;
    tmin = max(tmin, min(tx1, tx2));
    tmax = min(tmax, max(tx1, tx2));
  end
  if abs(dy) < 1e-12
    if py < ymin || py > ymax, t = NaN; return; end
  else
    ty1 = (ymin - py)/dy; ty2 = (ymax - py)/dy;
    tmin = max(tmin, min(ty1, ty2));
    tmax = min(tmax, max(ty1, ty2));
  end
  if tmax < 0 || tmin > tmax, t = NaN; return; end
  if tmin >= 0, t = tmin; else, t = tmax; end
  if t < 0, t = NaN; end
end

function L = updateRay(L, origin, res, pose, angleBody, r, zMax, l0, lOcc, lFree, lMin, lMax, alpha)
  s = worldToGrid(origin, res, pose(1), pose(2), size(L,2), size(L,1));
  if any(isnan(s)), return; end
  i0 = s(1); j0 = s(2);

  ang = pose(3) + angleBody;
  ex = pose(1) + r*cos(ang);
  ey = pose(2) + r*sin(ang);
  e = worldToGrid(origin, res, ex, ey, size(L,2), size(L,1));
  if any(isnan(e)), return; end
  i1 = e(1); j1 = e(2);

  cells = bresenham(i0, j0, i1, j1);
  if size(cells,1) <= 1, return; end

  hit = (r < (zMax - 0.5*alpha));
  if hit
    freeCells = cells(2:end-1, :);
  else
    freeCells = cells(2:end, :);
  end

  for idx = 1:size(freeCells,1)
    i = freeCells(idx,1); j = freeCells(idx,2);
    L(j,i) = min(max(L(j,i) + (lFree - l0), lMin), lMax);
  end

  if hit
    ie = cells(end,1); je = cells(end,2);
    L(je,ie) = min(max(L(je,ie) + (lOcc - l0), lMin), lMax);
  end
end

function ij = worldToGrid(origin, res, x, y, w, h)
  i = floor((x - origin(1))/res) + 1; % MATLAB 1-based
  j = floor((y - origin(2))/res) + 1;
  if i < 1 || i > w || j < 1 || j > h
    ij = [NaN, NaN];
  else
    ij = [i, j];
  end
end

function cells = bresenham(i0, j0, i1, j1)
  di = abs(i1 - i0);
  dj = abs(j1 - j0);
  si = 1; if i0 > i1, si = -1; end
  sj = 1; if j0 > j1, sj = -1; end
  err = di - dj;
  i = i0; j = j0;
  cells = [];
  while true
    cells(end+1,:) = [i, j]; %#ok<AGROW>
    if i == i1 && j == j1, break; end
    e2 = 2*err;
    if e2 > -dj
      err = err - dj;
      i = i + si;
    end
    if e2 < di
      err = err + di;
      j = j + sj;
    end
  end
end
