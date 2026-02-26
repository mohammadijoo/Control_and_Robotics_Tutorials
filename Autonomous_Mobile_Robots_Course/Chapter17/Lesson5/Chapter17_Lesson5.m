% Chapter17_Lesson5.m
% Autonomous Mobile Robots — Chapter 17 (Exploration and Active Mapping)
% Lesson 5 Lab: Autonomous Exploration in Unknown Map (single robot)
%
% Minimal MATLAB grid exploration demo:
% - Occupancy belief as log-odds grid
% - Frontier detection
% - Goal selection: score = w_ig*IG - w_cost*cost - w_risk*risk
%
% Run:
%   Chapter17_Lesson5

function Chapter17_Lesson5
  rng(7);
  W = 60; H = 45;
  obstacleProb = 0.22;

  occTrue = rand(H, W) < obstacleProb;
  occTrue(1,:) = true; occTrue(end,:) = true; occTrue(:,1) = true; occTrue(:,end) = true;

  % belief log-odds
  l = zeros(H, W);
  lOcc = logit(0.72);
  lFree = logit(0.28);
  lMin = logit(0.02);
  lMax = logit(0.98);

  % weights
  wIG = 1.0; wCost = 0.35; wRisk = 1.2;
  nRays = 48; rMax = 10;

  % initial pose (first free)
  [py, px] = find(~occTrue, 1, "first");
  pose = [px, py];

  % initial sensing
  l = senseUpdate(occTrue, l, pose, nRays, rMax, lFree, lOcc, lMin, lMax);

  travel = 0; steps = 0;
  maxSteps = 2500;

  for it = 1:maxSteps
    steps = steps + 1;
    cls = classifyMap(l);
    fronts = frontierCells(cls);
    if isempty(fronts)
      break;
    end

    % Sample K candidates
    fronts = fronts(randperm(size(fronts,1)), :);
    K = min(size(fronts,1), 60);

    bestScore = -1e18;
    bestGoal = fronts(1,:);

    for i = 1:K
      goal = fronts(i,:);
      [cost, path] = dijkstraGrid(cls, pose, goal);
      if ~isfinite(cost) || size(path,1) < 2
        continue;
      end

      ig = approxIG(cls, goal, rMax);
      risk = mean(probFromLogOdds(l(sub2ind(size(l), path(:,2), path(:,1)))));
      score = wIG*ig - wCost*cost - wRisk*risk;

      if score > bestScore
        bestScore = score;
        bestGoal = goal;
      end
    end

    [~, path] = dijkstraGrid(cls, pose, bestGoal);
    if size(path,1) < 2
      break;
    end
    next = path(2,:);

    if occTrue(next(2), next(1))
      l(next(2), next(1)) = lMax;
      continue;
    end

    pose = next;
    travel = travel + 1;
    l = senseUpdate(occTrue, l, pose, nRays, rMax, lFree, lOcc, lMin, lMax);
  end

  cls = classifyMap(l);
  knownFrac = mean(cls(:) ~= -1);

  fprintf("=== Exploration Summary (MATLAB) ===\n");
  fprintf("steps: %d\n", steps);
  fprintf("travel: %.2f\n", travel);
  fprintf("known_fraction: %.4f\n", knownFrac);
  fprintf("entropy_final: %.4f\n", totalEntropy(l));

  figure; imagesc(cls); axis image; title("Belief map: -1 unknown, 0 free, 1 occupied");
end

function v = logit(p)
  p = min(max(p, 1e-6), 1-1e-6);
  v = log(p/(1-p));
end

function p = probFromLogOdds(l)
  p = 1 ./ (1 + exp(-l));
end

function cls = classifyMap(l)
  p = probFromLogOdds(l);
  cls = -ones(size(l));
  cls(p >= 0.65) = 1;
  cls(p <= 0.35) = 0;
end

function H = totalEntropy(l)
  p = probFromLogOdds(l);
  p = min(max(p, 1e-12), 1-1e-12);
  H = sum(-p(:).*log(p(:)) - (1-p(:)).*log(1-p(:)));
end

function fronts = frontierCells(cls)
  [H,W] = size(cls);
  fronts = [];
  for y = 2:H-1
    for x = 2:W-1
      if cls(y,x) ~= -1, continue; end
      if cls(y,x+1)==0 || cls(y,x-1)==0 || cls(y+1,x)==0 || cls(y-1,x)==0
        fronts = [fronts; x, y]; %#ok<AGROW>
      end
    end
  end
end

function ig = approxIG(cls, goal, rMax)
  [H,W] = size(cls);
  x0 = goal(1); y0 = goal(2);
  ig = 0.0;
  for y = max(1, y0-rMax):min(H, y0+rMax)
    for x = max(1, x0-rMax):min(W, x0+rMax)
      if (x-x0)^2 + (y-y0)^2 <= rMax^2
        if cls(y,x) == -1
          ig = ig + log(2.0);
        end
      end
    end
  end
end

function l = senseUpdate(occTrue, l, pose, nRays, rMax, lFree, lOcc, lMin, lMax)
  fov = 2*pi;
  start = -fov/2;
  x = pose(1); y = pose(2);
  for k = 0:(nRays-1)
    theta = start + k/max(1,nRays-1)*fov;
    x1 = round(x + rMax*cos(theta));
    y1 = round(y + rMax*sin(theta));
    x1 = min(max(x1, 1), size(occTrue,2));
    y1 = min(max(y1, 1), size(occTrue,1));
    line = bresenhamLine(x, y, x1, y1);
    for i = 2:size(line,1)
      cx = line(i,1); cy = line(i,2);
      if occTrue(cy,cx)
        l(cy,cx) = min(max(l(cy,cx) + lOcc, lMin), lMax);
        break;
      else
        l(cy,cx) = min(max(l(cy,cx) + lFree, lMin), lMax);
      end
    end
  end
end

function line = bresenhamLine(x0,y0,x1,y1)
  dx = abs(x1-x0);
  dy = -abs(y1-y0);
  sx = 1; if x0 > x1, sx = -1; end
  sy = 1; if y0 > y1, sy = -1; end
  err = dx + dy;
  x = x0; y = y0;
  line = [x,y];
  while ~(x==x1 && y==y1)
    e2 = 2*err;
    if e2 >= dy
      err = err + dy;
      x = x + sx;
    end
    if e2 <= dx
      err = err + dx;
      y = y + sy;
    end
    line = [line; x, y]; %#ok<AGROW>
  end
end

function [cost, path] = dijkstraGrid(cls, start, goal)
  % Dijkstra on 4-neighbor grid with:
  % free cost 1, unknown cost 3, occupied forbidden
  [H,W] = size(cls);
  sx = start(1); sy = start(2);
  gx = goal(1); gy = goal(2);

  INF = 1e18;
  dist = INF*ones(H,W);
  parentX = zeros(H,W);
  parentY = zeros(H,W);
  visited = false(H,W);

  dist(sy,sx) = 0;
  % priority queue as simple list (for teaching/demo; O(n^2))
  for iter = 1:(H*W)
    % extract min
    minVal = INF; mx = -1; my = -1;
    for y = 1:H
      for x = 1:W
        if ~visited(y,x) && dist(y,x) < minVal
          minVal = dist(y,x); mx = x; my = y;
        end
      end
    end
    if mx < 0, break; end
    visited(my,mx) = true;
    if mx==gx && my==gy, break; end

    neigh = [mx+1,my; mx-1,my; mx,my+1; mx,my-1];
    for i = 1:4
      nx = neigh(i,1); ny = neigh(i,2);
      if nx<1||nx>W||ny<1||ny>H, continue; end
      if cls(ny,nx) == 1, continue; end
      step = 3; if cls(ny,nx) == 0, step = 1; end
      nd = dist(my,mx) + step;
      if nd < dist(ny,nx)
        dist(ny,nx) = nd;
        parentX(ny,nx) = mx;
        parentY(ny,nx) = my;
      end
    end
  end

  cost = dist(gy,gx);
  if ~isfinite(cost) || cost > 1e17
    path = zeros(0,2);
    return;
  end
  % recover path
  path = [gx,gy];
  cx = gx; cy = gy;
  while ~(cx==sx && cy==sy)
    px = parentX(cy,cx); py = parentY(cy,cx);
    if px==0 && py==0
      path = zeros(0,2); return;
    end
    cx = px; cy = py;
    path = [cx,cy; path]; %#ok<AGROW>
  end
end
