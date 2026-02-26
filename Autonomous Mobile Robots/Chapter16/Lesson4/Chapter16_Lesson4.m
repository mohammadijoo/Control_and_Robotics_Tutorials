% Chapter16_Lesson4.m
% Prediction-Aware Local Navigation (sampling MPC with chance-constraint penalty)
% Minimal MATLAB script (no toolboxes required).

function Chapter16_Lesson4()
  x0 = [0; 0; 0];           % [x;y;theta]
  goal = [6; 0];

  % Two moving obstacles (mean state + simple pos covariance)
  tracks(1).mu = [3; 1; 0; -0.6];
  tracks(1).Sigma = diag([0.15^2, 0.15^2, 0.3^2, 0.3^2]);
  tracks(1).r = 0.35;

  tracks(2).mu = [4; -1.2; 0; 0.7];
  tracks(2).Sigma = diag([0.15^2, 0.15^2, 0.3^2, 0.3^2]);
  tracks(2).r = 0.35;

  dt = 0.1; N = 25;
  delta = 0.01;
  robot_r = 0.25;

  v_range = [0, 1.2];
  w_range = [-1.8, 1.8];
  n_v = 13; n_w = 31;

  bestJ = inf; bestU = [0;0];
  for i = 1:n_v
    v = v_range(1) + (v_range(2)-v_range(1))*(i-1)/(n_v-1);
    for j = 1:n_w
      w = w_range(1) + (w_range(2)-w_range(1))*(j-1)/(n_w-1);
      J = rollout_cost(x0, [v; w], goal, tracks, dt, N, delta, robot_r);
      if J < bestJ
        bestJ = J;
        bestU = [v; w];
      end
    end
  end

  fprintf('Best control u* = [v,w] = [%.3f, %.3f], cost = %.3f\\n', bestU(1), bestU(2), bestJ);
end

function x1 = unicycle_step(x, u, dt)
  v = u(1); w = u(2);
  x1 = zeros(3,1);
  x1(1) = x(1) + dt*v*cos(x(3));
  x1(2) = x(2) + dt*v*sin(x(3));
  x1(3) = wrapToPi(x(3) + dt*w);
end

function tracks1 = predict_tracks(tracks, dt)
  % constant velocity prediction for mean, covariance inflation
  F = [1 0 dt 0;
       0 1 0 dt;
       0 0 1 0;
       0 0 0 1];
  Q = diag([0.05^2, 0.05^2, 0.2^2, 0.2^2]);
  tracks1 = tracks;
  for k = 1:numel(tracks)
    tracks1(k).mu = F*tracks(k).mu;
    tracks1(k).Sigma = F*tracks(k).Sigma*F' + Q;
  end
end

function pen = chance_penalty_point(p, mu_o, Sigma_o, R_safe, delta)
  Sigma = Sigma_o(1:2,1:2);
  mu_r = p - mu_o(1:2);
  dist = norm(mu_r) + 1e-9;
  n = mu_r / dist;

  m = dist;
  s2 = n'*Sigma*n;
  s = sqrt(max(0,s2) + 1e-12);

  z = z_value(delta);
  margin = z*s;
  violation = max(0, margin - (m - R_safe));
  pen = violation^2;
end

function z = z_value(delta)
  % z_{1-delta} table
  if abs(delta-0.1) < 1e-12, z = 1.281551565545;
  elseif abs(delta-0.05) < 1e-12, z = 1.644853626951;
  elseif abs(delta-0.02) < 1e-12, z = 2.053748910631;
  elseif abs(delta-0.01) < 1e-12, z = 2.326347874041;
  elseif abs(delta-0.005) < 1e-12, z = 2.575829303549;
  elseif abs(delta-0.001) < 1e-12, z = 3.090232306168;
  else, z = 2.326347874041; % fallback
  end
end

function J = rollout_cost(x0, u, goal, tracks0, dt, N, delta, robot_r)
  w_goal = 1.0; w_ctrl = 0.05; w_risk = 10.0;

  x = x0; tracks = tracks0;
  J = 0;
  for k = 1:N
    tracks = predict_tracks(tracks, dt);
    x = unicycle_step(x, u, dt);

    e = x(1:2) - goal;
    J = J + w_goal*(e'*e) + w_ctrl*(u'*u);

    risk = 0;
    for j = 1:numel(tracks)
      R_safe = robot_r + tracks(j).r;
      risk = risk + chance_penalty_point(x(1:2), tracks(j).mu, tracks(j).Sigma, R_safe, delta);
    end
    J = J + w_risk*risk;
  end
end
