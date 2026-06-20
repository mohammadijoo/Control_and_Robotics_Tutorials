% Chapter16_Lesson5.m
% Autonomous Mobile Robots — Chapter 16, Lesson 5
% Lab: Navigate Through Moving Crowds
%
% MATLAB implementation (script + local functions):
%   - Crowd: constant-velocity discs (with small jitter)
%   - Robot: unicycle
%   - Planner: sampling-based crowd-aware DWA (TTC + social discomfort)
%
% Optional: run build_simulink_model() to generate a simple Simulink diagram
% that integrates a unicycle plant with a MATLAB Function controller block.
%
% Tested conceptually with base MATLAB; no toolboxes required for the core sim.

function Chapter16_Lesson5()
  rng(4);

  P = params();
  dt = P.dt;

  humans = make_crossing_crowd(12, 0.9);

  robot.p = [-8.0; -8.0];
  robot.th = pi/4;
  robot.v = 0.0;
  robot.w = 0.0;
  robot.r = 0.32;

  goal = [8.0; 8.0];

  T_max = 70.0;
  steps = floor(T_max / dt);

  traj = zeros(steps, 6); % t x y th v w
  K = 0;

  for step = 1:steps
    t = (step-1) * dt;

    [v_cmd, w_cmd] = plan_control(robot, goal, humans, P);
    robot.v = v_cmd;
    robot.w = w_cmd;

    [robot.p, robot.th] = simulate_unicycle(robot.p, robot.th, robot.v, robot.w, dt);

    humans = update_humans(humans, dt, 9.0, 0.01);

    K = K + 1;
    traj(K,:) = [t, robot.p(1), robot.p(2), robot.th, robot.v, robot.w];

    if norm(goal - robot.p) < 0.5
      disp('Result: reached goal');
      break;
    end

    if any_collide(robot, humans)
      disp('Result: collision');
      break;
    end
  end

  traj = traj(1:K,:);

  figure; hold on; axis equal; grid on;
  plot(traj(:,2), traj(:,3), 'LineWidth', 1.2);
  plot(goal(1), goal(2), '*', 'MarkerSize', 10);
  title('Crowd navigation (MATLAB sampling-based crowd-aware DWA)');
  xlabel('x'); ylabel('y');

  % final human positions
  hx = arrayfun(@(h) h.p(1), humans);
  hy = arrayfun(@(h) h.p(2), humans);
  scatter(hx, hy, 40, 'filled');

  % save a CSV log
  writematrix(traj, 'robot_traj_matlab.csv');
  disp('Wrote robot_traj_matlab.csv');

  % Uncomment to build a simple Simulink model:
  % build_simulink_model();
end

% -----------------------------
% Parameters
% -----------------------------
function P = params()
  P.dt = 0.1;
  P.horizon = 2.5;
  P.v_max = 1.2;
  P.w_max = 1.8;
  P.a_v = 1.0;
  P.a_w = 2.5;

  P.w_goal = 5.0;
  P.w_clear = 2.0;
  P.w_ttc = 4.0;
  P.w_soc = 2.0;
  P.w_vel = 0.5;
  P.w_turn = 0.1;
  P.w_smooth = 0.2;

  P.sigma_front = 0.9;
  P.sigma_side = 0.6;
  P.sigma_back = 0.5;
end

% -----------------------------
% Crowd scenario
% -----------------------------
function humans = make_crossing_crowd(n_per_stream, speed)
  r = 0.25;

  % left -> right
  ys = -2 + 4 * rand(n_per_stream,1);
  xs = -7 + 3.5 * rand(n_per_stream,1);

  humans = repmat(struct('p',[0;0],'v',[0;0],'r',r), 2*n_per_stream, 1);
  idx = 1;
  for i=1:n_per_stream
    humans(idx).p = [xs(i); ys(i)];
    humans(idx).v = [speed; 0];
    humans(idx).r = r;
    idx = idx + 1;
  end

  % bottom -> top
  xs = -2 + 4 * rand(n_per_stream,1);
  ys = -7 + 3.5 * rand(n_per_stream,1);

  for i=1:n_per_stream
    humans(idx).p = [xs(i); ys(i)];
    humans(idx).v = [0; speed];
    humans(idx).r = r;
    idx = idx + 1;
  end
end

function humans = update_humans(humans, dt, arena, noise_std)
  for i=1:numel(humans)
    if noise_std > 0
      humans(i).v = humans(i).v + noise_std * randn(2,1);
      sp = norm(humans(i).v);
      if sp > 1e-6
        sp2 = min(sp, 1.5);
        humans(i).v = humans(i).v / sp * sp2;
      end
    end

    humans(i).p = humans(i).p + humans(i).v * dt;

    for k=1:2
      if humans(i).p(k) < -arena
        humans(i).p(k) = -arena;
        humans(i).v(k) = abs(humans(i).v(k));
      end
      if humans(i).p(k) > arena
        humans(i).p(k) = arena;
        humans(i).v(k) = -abs(humans(i).v(k));
      end
    end
  end
end

% -----------------------------
% Planner and helpers
% -----------------------------
function [p2, th2] = simulate_unicycle(p, th, v, w, dt)
  p2 = p + [v*cos(th); v*sin(th)] * dt;
  th2 = wrap_angle(th + w*dt);
end

function a = wrap_angle(a)
  a = mod(a + pi, 2*pi);
  if a < 0, a = a + 2*pi; end
  a = a - pi;
end

function ttc = ttc_discs(p_rel, v_rel, R)
  eps = 1e-9;
  a = dot(v_rel, v_rel);
  b = 2 * dot(p_rel, v_rel);
  c = dot(p_rel, p_rel) - R^2;

  if c <= 0
    ttc = 0;
    return;
  end
  if a <= eps
    ttc = inf;
    return;
  end

  disc = b^2 - 4*a*c;
  if disc < 0
    ttc = inf;
    return;
  end

  s = sqrt(disc);
  t1 = (-b - s) / (2*a);
  t2 = (-b + s) / (2*a);

  if t1 >= 0
    ttc = t1;
  elseif t2 >= 0
    ttc = t2;
  else
    ttc = inf;
  end
end

function c = social_cost(rp, humans, P)
  c = 0;
  for i=1:numel(humans)
    hv = humans(i).v;
    vnorm = norm(hv);
    if vnorm < 1e-6
      phi = 0;
    else
      phi = atan2(hv(2), hv(1));
    end
    R = [cos(-phi) -sin(-phi); sin(-phi) cos(-phi)]; % world->human frame
    rel_h = R * (rp - humans(i).p);
    dx = rel_h(1); dy = rel_h(2);

    if dx >= 0
      sx = P.sigma_front;
    else
      sx = P.sigma_back;
    end
    sy = P.sigma_side;

    q = 0.5 * ((dx/sx)^2 + (dy/sy)^2);
    c = c + exp(-q);
  end
end

function [v_best, w_best] = plan_control(robot, goal, humans, P)
  dt = P.dt;
  N = max(1, round(P.horizon / dt));

  v_lo = clamp(robot.v - P.a_v*dt, 0, P.v_max);
  v_hi = clamp(robot.v + P.a_v*dt, 0, P.v_max);
  w_lo = clamp(robot.w - P.a_w*dt, -P.w_max, P.w_max);
  w_hi = clamp(robot.w + P.a_w*dt, -P.w_max, P.w_max);

  d_goal = norm(goal - robot.p);
  if d_goal > 1
    v_pref = P.v_max;
  else
    v_pref = P.v_max * d_goal;
  end

  v_samples = linspace(v_lo, v_hi, 9);
  w_samples = linspace(w_lo, w_hi, 11);

  best_cost = inf;
  v_best = 0; w_best = 0;

  for vi=1:numel(v_samples)
    for wi=1:numel(w_samples)
      v = v_samples(vi);
      w = w_samples(wi);

      p = robot.p;
      th = robot.th;
      ttc_min = inf;
      clear_min = inf;
      soc_sum = 0;

      for k=1:N
        [p, th] = simulate_unicycle(p, th, v, w, dt);
        t = k * dt;

        humans_t = humans;
        for j=1:numel(humans_t)
          humans_t(j).p = humans_t(j).p + humans_t(j).v * t;
        end

        for j=1:numel(humans_t)
          rel = p - humans_t(j).p;
          dist = norm(rel) - (robot.r + humans_t(j).r);
          clear_min = min(clear_min, dist);

          v_r = [v*cos(th); v*sin(th)];
          v_rel = v_r - humans_t(j).v;
          ttc = ttc_discs(rel, v_rel, robot.r + humans_t(j).r);
          ttc_min = min(ttc_min, ttc);
        end
        soc_sum = soc_sum + social_cost(p, humans_t, P);
      end

      d_term = norm(goal - p);
      eps = 1e-6;

      c_goal = P.w_goal * d_term;
      c_clear = P.w_clear * (1 / (clear_min + eps));
      c_ttc  = P.w_ttc  * (1 / (ttc_min + eps));
      c_soc  = P.w_soc  * (soc_sum / N);
      c_vel  = P.w_vel  * (v_pref - v)^2;
      c_turn = P.w_turn * (w^2);
      c_smooth = P.w_smooth * ((v-robot.v)^2 + 0.1*(w-robot.w)^2);

      if clear_min <= 0
        continue;
      end

      cost = c_goal + c_clear + c_ttc + c_soc + c_vel + c_turn + c_smooth;
      if cost < best_cost
        best_cost = cost;
        v_best = v;
        w_best = w;
      end
    end
  end
end

function x = clamp(x, lo, hi)
  x = min(max(x, lo), hi);
end

function c = any_collide(robot, humans)
  c = false;
  for i=1:numel(humans)
    if norm(robot.p - humans(i).p) <= (robot.r + humans(i).r)
      c = true;
      return;
    end
  end
end

% -----------------------------
% Simulink model builder (optional)
% -----------------------------
function build_simulink_model()
  mdl = 'Chapter16_Lesson5_Simulink';
  if bdIsLoaded(mdl)
    close_system(mdl,0);
  end
  new_system(mdl);
  open_system(mdl);

  % Create a basic unicycle plant xdot = v cos(th), ydot = v sin(th), thdot = w
  add_block('simulink/Sources/Constant', [mdl '/goal'], 'Value', '[8;8]');
  add_block('simulink/Sources/Clock', [mdl '/t']);
  add_block('simulink/Continuous/Integrator', [mdl '/Int_x']);
  add_block('simulink/Continuous/Integrator', [mdl '/Int_y']);
  add_block('simulink/Continuous/Integrator', [mdl '/Int_th']);
  add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Controller']);

  % Note: wiring a complete controller and crowd generator is lengthy; this
  % function demonstrates programmatic creation of a Simulink scaffold.
  % Students can fill in human states as signals and implement planning in the
  % MATLAB Function block.

  set_param([mdl '/Int_x'], 'InitialCondition', '-8');
  set_param([mdl '/Int_y'], 'InitialCondition', '-8');
  set_param([mdl '/Int_th'], 'InitialCondition', 'pi/4');

  save_system(mdl);
  disp(['Saved Simulink model: ' mdl '.slx']);
end
