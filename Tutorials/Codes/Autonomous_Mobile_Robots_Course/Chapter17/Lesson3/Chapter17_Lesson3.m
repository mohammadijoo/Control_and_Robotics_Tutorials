% Chapter17_Lesson3.m
% Autonomous Mobile Robots (Control Engineering) — Chapter 17, Lesson 3
% Next-Best-View (NBV) Strategies — MATLAB/Simulink-oriented reference
%
% This script:
%   1) Builds a belief occupancy grid (probabilities per cell)
%   2) Computes expected information gain (EIG) for candidate viewpoints
%   3) Chooses the NBV via U(v) = EIG(v) - lambda * travel_cost
%   4) (Optional) Creates a Simulink skeleton model programmatically
%
% Requirements: base MATLAB. (Robotics System Toolbox can be used for real robots.)

function Chapter17_Lesson3()
  rng(7);

  W = 30; H = 18;
  belief = 0.5 * ones(H, W); % P(occupied), unknown prior

  % Sensor model parameters
  p_hit = 0.85;   % P(z=occ | m=occ)
  p_false = 0.15; % P(z=occ | m=free)

  % Current pose (grid coords)
  cx = 2; cy = 2; heading = 0.0;

  % Candidates
  N = 120;
  cand = [randi([1 W], N, 1), randi([1 H], N, 1), 2*pi*rand(N,1)]; % [x y theta]

  lambda = 0.22;

  bestU = -1e18;
  best = [cx cy heading 0 bestU]; % [x y theta IG U]

  for i = 1:N
    vx = cand(i,1); vy = cand(i,2); th = cand(i,3);
    ig = expectedIgView(belief, vx, vy, th, p_hit, p_false, pi, 45, 10, 0.70);
    cost = hypot(vx - cx, vy - cy);
    u = ig - lambda * cost;
    if u > bestU
      bestU = u;
      best = [vx vy th ig u];
    end
  end

  fprintf('NBV = (%d,%d), heading=%.3f, IG=%.3f, U=%.3f\n', best(1), best(2), best(3), best(4), best(5));
  fprintf('(This demo only selects a view; a real system would plan & move, then update belief.)\n');

  % --- Optional: create a Simulink skeleton model (no .slx included in this package) ---
  % createSimulinkSkeleton();
end

function H = bernoulliEntropy(p)
  eps = 1e-12;
  p = min(max(p, eps), 1-eps);
  H = -p*log(p) - (1-p)*log(1-p);
end

function ppost = posterior(p, zOcc, p_hit, p_false)
  eps = 1e-12;
  p = min(max(p, eps), 1-eps);
  if zOcc
    num = p_hit * p;
    den = p_hit * p + p_false * (1-p);
  else
    num = (1-p_hit) * p;
    den = (1-p_hit) * p + (1-p_false) * (1-p);
  end
  ppost = min(max(num/den, eps), 1-eps);
end

function E = expectedPosteriorEntropy(p, p_hit, p_false)
  pzOcc = p_hit * p + p_false * (1-p);
  pzFree = 1 - pzOcc;
  pPostOcc = posterior(p, true,  p_hit, p_false);
  pPostFree = posterior(p, false, p_hit, p_false);
  E = pzOcc * bernoulliEntropy(pPostOcc) + pzFree * bernoulliEntropy(pPostFree);
end

function ig = expectedInformationGain(p, p_hit, p_false)
  ig = bernoulliEntropy(p) - expectedPosteriorEntropy(p, p_hit, p_false);
end

function pts = bresenham(x0, y0, x1, y1)
  dx = abs(x1 - x0);
  dy = -abs(y1 - y0);
  sx = 1; if x0 >= x1, sx = -1; end
  sy = 1; if y0 >= y1, sy = -1; end
  err = dx + dy;
  x = x0; y = y0;
  pts = [];
  while true
    pts(end+1,:) = [x y]; %#ok<AGROW>
    if x == x1 && y == y1
      break;
    end
    e2 = 2*err;
    if e2 >= dy
      err = err + dy;
      x = x + sx;
    end
    if e2 <= dx
      err = err + dx;
      y = y + sy;
    end
  end
end

function [ex, ey] = rayEndpoint(cx, cy, theta, r)
  ex = round(cx + r*cos(theta));
  ey = round(cy + r*sin(theta));
end

function total = expectedIgView(belief, cx, cy, heading, p_hit, p_false, fov, nRays, maxRange, occStop)
  [H, W] = size(belief);
  if cx < 1 || cx > W || cy < 1 || cy > H
    total = -1e9;
    return;
  end
  total = 0.0;
  angles = linspace(heading - 0.5*fov, heading + 0.5*fov, nRays);
  for a = angles
    [ex, ey] = rayEndpoint(cx, cy, a, maxRange);
    line = bresenham(cx, cy, ex, ey);
    for k = 2:size(line,1)
      x = line(k,1); y = line(k,2);
      if x < 1 || x > W || y < 1 || y > H
        break;
      end
      pOcc = belief(y,x);
      total = total + expectedInformationGain(pOcc, p_hit, p_false);
      if pOcc > occStop
        break;
      end
    end
  end
end

function createSimulinkSkeleton()
  % This creates a basic Simulink model showing where NBV selection would live.
  % Students can extend it by adding a MATLAB Function block that calls expectedIgView().
  mdl = 'NBV_Skeleton_Chapter17_Lesson3';
  if bdIsLoaded(mdl); close_system(mdl, 0); end
  new_system(mdl);
  open_system(mdl);

  add_block('simulink/Sources/Constant', [mdl '/BeliefGrid'], 'Value', '0.5');
  add_block('simulink/Sources/Constant', [mdl '/CurrentPose'], 'Value', '[2 2 0]');
  add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/NBV_Selector']);
  add_block('simulink/Sinks/Display', [mdl '/BestView']);

  set_param([mdl '/NBV_Selector'], 'Position', [250 100 450 170]);
  set_param([mdl '/BestView'], 'Position', [520 115 650 155]);

  add_line(mdl, 'BeliefGrid/1', 'NBV_Selector/1');
  add_line(mdl, 'CurrentPose/1', 'NBV_Selector/2');
  add_line(mdl, 'NBV_Selector/1', 'BestView/1');

  save_system(mdl);
  fprintf('Created Simulink skeleton model: %s.slx\n', mdl);
end
