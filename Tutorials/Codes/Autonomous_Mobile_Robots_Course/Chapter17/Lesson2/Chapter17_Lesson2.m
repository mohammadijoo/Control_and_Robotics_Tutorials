% Chapter 17 - Lesson 2: Information Gain and Entropy Reduction
% Autonomous Mobile Robots (Control Engineering)
%
% MATLAB implementation for:
% - Bernoulli (occupancy) entropy
% - Per-cell information gain for a binary sensor model
% - Approx expected info gain for a viewpoint using ray stepping
%
% Simulink note:
%   You can place expected_info_gain_view(...) into a MATLAB Function block
%   to score candidate viewpoints generated elsewhere.
%
% Run:
%   Chapter17_Lesson2

function Chapter17_Lesson2()
  rng(0);

  % Build a grid
  g.width = 120; g.height = 120; g.resolution = 0.1;
  g.origin_x = -6.0; g.origin_y = -6.0;
  g.p = 0.5 * ones(g.height, g.width);

  % Random free-ish and occupied-ish patches
  for k = 1:4000
    ix = randi(g.width); iy = randi(g.height);
    g.p(iy, ix) = clip(0.15 + 0.10 * randn(), 0.01, 0.99);
  end
  for k = 1:1200
    ix = randi(g.width); iy = randi(g.height);
    g.p(iy, ix) = clip(0.85 + 0.08 * randn(), 0.01, 0.99);
  end

  H0 = map_entropy(g.p);
  fprintf('Initial map entropy (nats): %.3f\n', H0);

  poses = [
    0.0, 0.0, 0.0;
    -2.0, 1.5, 0.7;
    2.0, -1.0, -1.2
  ];

  for i = 1:size(poses,1)
    pose = poses(i,:);
    ig = expected_info_gain_view(g, pose, 180.0, 181, 6.0, 0.85, 0.15, [0.4, 0.6], 0.75);
    fprintf('Pose=(%.2f, %.2f, %.2f)  Expected IG≈ %.3f nats\n', pose(1), pose(2), pose(3), ig);
  end
end

function y = clip(x, lo, hi)
  y = min(max(x, lo), hi);
end

function H = bernoulli_entropy(p)
  eps = 1e-12;
  p = clip(p, eps, 1-eps);
  H = -p .* log(p) - (1-p) .* log(1-p);
end

function IG = info_gain_cell(p, p_hit, p_false)
  Hprior = bernoulli_entropy(p);

  PzOcc = p_hit * p + p_false * (1 - p);
  PzFree = 1 - PzOcc;

  eps = 1e-15;
  PzOcc = clip(PzOcc, eps, 1-eps);
  PzFree = clip(PzFree, eps, 1-eps);

  pPostOcc  = (p_hit * p) / PzOcc;
  pPostFree = ((1 - p_hit) * p) / PzFree;

  Hpost = PzOcc * bernoulli_entropy(pPostOcc) + PzFree * bernoulli_entropy(pPostFree);
  IG = Hprior - Hpost;
end

function Hm = map_entropy(P)
  Hm = sum(bernoulli_entropy(P), 'all');
end

function [ix, iy] = world_to_grid(g, x, y)
  ix = floor((x - g.origin_x) / g.resolution) + 1; % MATLAB 1-based
  iy = floor((y - g.origin_y) / g.resolution) + 1;
end

function in = in_bounds(g, ix, iy)
  in = (ix >= 1) && (ix <= g.width) && (iy >= 1) && (iy <= g.height);
end

function cells = ray_cast_cells(g, x0, y0, theta, max_range)
  step = 0.5 * g.resolution;
  cells = zeros(0,2);
  seen = false(g.height, g.width);

  t = 0.0;
  while t <= max_range
    x = x0 + t * cos(theta);
    y = y0 + t * sin(theta);
    [ix, iy] = world_to_grid(g, x, y);
    if ~in_bounds(g, ix, iy)
      break;
    end
    if ~seen(iy, ix)
      seen(iy, ix) = true;
      cells(end+1,:) = [ix, iy]; %#ok<AGROW>
    end
    t = t + step;
  end
end

function ig_total = expected_info_gain_view(g, pose, fov_deg, n_rays, max_range, p_hit, p_false, unknown_band, occ_stop_threshold)
  x = pose(1); y = pose(2); yaw = pose(3);
  fov = deg2rad(fov_deg);

  if n_rays < 2
    angles = 0;
  else
    angles = linspace(-0.5*fov, 0.5*fov, n_rays);
  end

  lo_u = unknown_band(1); hi_u = unknown_band(2);
  visited = false(g.height, g.width);
  ig_total = 0.0;

  for a = angles
    theta = yaw + a;
    cells = ray_cast_cells(g, x, y, theta, max_range);

    for k = 1:size(cells,1)
      ix = cells(k,1); iy = cells(k,2);
      if visited(iy, ix)
        continue;
      end
      visited(iy, ix) = true;

      p = g.p(iy, ix);

      if p >= occ_stop_threshold
        break; % occlusion
      end

      if (p >= lo_u) && (p <= hi_u)
        ig_total = ig_total + info_gain_cell(p, p_hit, p_false);
      end
    end
  end
end
