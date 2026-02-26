% Chapter15_Lesson5.m
% Lab: Compare Local Planners in Dense Obstacles (DWA vs simplified TEB-proxy)
% Run:
%   ComparePlanners(30, 0);

function ComparePlanners(trials, seed0)
  if nargin < 1, trials = 30; end
  if nargin < 2, seed0 = 0; end

  prm.R=0.25; prm.vmax=0.9; prm.wmax=1.6; prm.a=1.2; prm.alpha=2.5;
  prm.dt=0.1; prm.T=2.0; prm.goal_tol=0.25; prm.max_steps=800; prm.n_obs=45;

  fprintf('\nDWA: '); disp(Summarize(RunMany('dwa', trials, seed0, prm)));
  fprintf('\nTEB: '); disp(Summarize(RunMany('teb', trials, seed0, prm)));
end

function R = RunMany(name, trials, seed0, prm)
  R = repmat(struct('success',false,'collision',false,'time',0,'L',0,'cmin',inf,'w2',0), trials, 1);
  for i=1:trials
    R(i) = RunOne(name, seed0 + (i-1), prm);
  end
end

function S = Summarize(R)
  n = numel(R);
  succ = sum([R.success]); col = sum([R.collision]);
  S.trials = n;
  S.success_rate = succ/n;
  S.collision_rate = col/n;
  S.time_mean = mean([R.time]);
  S.L_mean = mean([R.L]);
  S.cmin_mean = mean([R.cmin]);
  S.w2_mean = mean([R.w2]);
end

% ---------- World / geometry ----------
function y = corridor_y(x), y = 0.9*sin(0.6*x); end

function path = reference_path()
  xs = linspace(0.5, 11.5, 120)';
  ys = arrayfun(@corridor_y, xs);
  path = [xs, ys];
end

function obs = sample_dense_world(n_obs, seed)
  rng(seed);
  obs = zeros(0,3);
  tries = 0;
  while size(obs,1) < n_obs && tries < 20000
    tries = tries + 1;
    x = 0.5 + 11.0*rand();
    y = -2.5 + 5.0*rand();
    r = 0.12 + 0.20*rand();
    if abs(y - corridor_y(x)) < 0.55 + r, continue; end
    ok = true;
    for k=1:size(obs,1)
      dx = x-obs(k,1); dy = y-obs(k,2);
      rr = r + obs(k,3) + 0.05;
      if dx*dx + dy*dy < rr*rr, ok=false; break; end
    end
    if ok, obs(end+1,:) = [x,y,r]; end %#ok<AGROW>
  end
end

function c = clearance(p, obs, R)
  d = inf;
  for i=1:size(obs,1)
    d = min(d, hypot(p(1)-obs(i,1), p(2)-obs(i,2)) - obs(i,3) - R);
  end
  c = d;
end

function a = wrap(a), a = mod(a+pi, 2*pi) - pi; end

% ---------- Unicycle ----------
function x2 = step_unicycle(x, v, w, dt)
  x2 = [x(1) + v*cos(x(3))*dt;
        x(2) + v*sin(x(3))*dt;
        wrap(x(3) + w*dt)];
end

% ---------- DWA ----------
function [xf, mc] = rollout_min_clear(x, v, w, obs, prm)
  steps = max(1, round(prm.T/prm.dt));
  mc = inf; xf = x;
  for k=1:steps
    xf = step_unicycle(xf, v, w, prm.dt);
    mc = min(mc, clearance(xf(1:2)', obs, prm.R));
    if mc <= 0, break; end
  end
end

function [vbest, wbest] = dwa_command(x, v0, w0, path, obs, prm)
  vmin = max(0, v0 - prm.a*prm.dt); vmax = min(prm.vmax, v0 + prm.a*prm.dt);
  wmin = max(-prm.wmax, w0 - prm.alpha*prm.dt); wmax = min(prm.wmax, w0 + prm.alpha*prm.dt);

  goal = path(end,:);
  bestJ = -1e18; vbest = 0; wbest = 0;

  for v = linspace(vmin, vmax, 9)
    for w = linspace(wmin, wmax, 17)
      [xf, mc] = rollout_min_clear(x, v, w, obs, prm);
      if mc <= prm.R + 0.05, continue; end
      goalDist = norm(xf(1:2)' - goal);
      J = -0.6*goalDist + 1.8*mc + 0.4*(v/(prm.vmax+1e-9)) - 0.12*w*w;
      if J > bestJ, bestJ = J; vbest = v; wbest = w; end
    end
  end
end

% ---------- TEB-proxy ----------
function pts = optimize_band(x, obs, prm, N, bandLen, iters, step)
  if nargin < 4, N=12; end
  if nargin < 5, bandLen=2.6; end
  if nargin < 6, iters=20; end
  if nargin < 7, step=0.12; end

  xs = linspace(x(1), x(1)+bandLen, N)';
  pts = [xs, arrayfun(@corridor_y, xs)];
  pts(1,:) = x(1:2)';

  for it=1:iters
    g = zeros(size(pts));

    g(2:end-1,:) = g(2:end-1,:) + 0.35*(2*pts(2:end-1,:) - pts(1:end-2,:) - pts(3:end,:));

    for i=2:N
      bestD = inf; bestU = [0,0];
      for k=1:size(obs,1)
        dvec = pts(i,:) - obs(k,1:2);
        n = norm(dvec) + 1e-12;
        d = n - obs(k,3) - prm.R;
        if d < bestD, bestD = d; bestU = dvec/n; end
      end
      if bestD < 0.9
        phi = exp(-4.5*(bestD - 0.9));
        g(i,:) = g(i,:) + 1.8*(-4.5*phi)*bestU;
      end
    end

    g(2:end,:) = g(2:end,:) + 0.25*(pts(2:end,:) - pts(1:end-1,:));

    pts(2:end,:) = pts(2:end,:) - step*g(2:end,:);
    pts(1,:) = x(1:2)';
  end
end

function [vcmd, wcmd] = teb_command(x, v0, w0, path, obs, prm) %#ok<INUSD>
  pts = optimize_band(x, obs, prm);
  d = pts(2,:) - pts(1,:);
  heading = atan2(d(2), d(1));
  herr = wrap(heading - x(3));

  wcmd = max(-prm.wmax, min(prm.wmax, 2.2*herr));
  vcmd = max(0, min(prm.vmax, 0.8*(norm(d)/prm.dt)));
  vcmd = vcmd / (1 + 1.2*abs(wcmd));

  vcmd = max(max(0, v0 - prm.a*prm.dt), min(min(prm.vmax, v0 + prm.a*prm.dt), vcmd));
  wcmd = max(w0 - prm.alpha*prm.dt, min(w0 + prm.alpha*prm.dt, wcmd));
end

% ---------- Simulation ----------
function r = RunOne(planner, seed, prm)
  obs = sample_dense_world(prm.n_obs, seed);
  path = reference_path();
  goal = path(end,:);

  x = [0.6;0.0;0.0]; v=0; w=0;
  L=0; cmin=inf; w2=0;

  for k=1:prm.max_steps
    c = clearance(x(1:2)', obs, prm.R);
    cmin = min(cmin, c);
    if c <= 0
      r = struct('success',false,'collision',true,'time',(k-1)*prm.dt,'L',L,'cmin',cmin,'w2',w2);
      return;
    end
    if norm(x(1:2)' - goal) <= prm.goal_tol
      r = struct('success',true,'collision',false,'time',(k-1)*prm.dt,'L',L,'cmin',cmin,'w2',w2);
      return;
    end

    if strcmpi(planner,'teb')
      [vcmd,wcmd] = teb_command(x,v,w,path,obs,prm);
    else
      [vcmd,wcmd] = dwa_command(x,v,w,path,obs,prm);
    end

    x2 = step_unicycle(x, vcmd, wcmd, prm.dt);
    L = L + norm(x2(1:2)-x(1:2));
    w2 = w2 + (wcmd*wcmd)*prm.dt;
    x = x2; v=vcmd; w=wcmd;
  end

  r = struct('success',false,'collision',false,'time',prm.max_steps*prm.dt,'L',L,'cmin',cmin,'w2',w2);
end

% ---------------- Optional: Simulink Harness Builder ----------------
% BuildSimulinkHarness('AMR_UnicycleHarness');

function BuildSimulinkHarness(modelName)
  if nargin < 1, modelName = 'AMR_UnicycleHarness'; end
  if bdIsLoaded(modelName), close_system(modelName,0); end
  new_system(modelName); open_system(modelName);
  set_param(modelName, 'StopTime', '20');

  add_block('simulink/Sources/From Workspace', [modelName '/Vin'], 'VariableName', 'Vin', 'Position', [30 60 140 90]);
  add_block('simulink/Sources/From Workspace', [modelName '/Win'], 'VariableName', 'Win', 'Position', [30 140 140 170]);

  add_block('simulink/Math Operations/Trigonometric Function', [modelName '/cos'], 'Operator', 'cos', 'Position', [220 40 270 80]);
  add_block('simulink/Math Operations/Trigonometric Function', [modelName '/sin'], 'Operator', 'sin', 'Position', [220 110 270 150]);
  add_block('simulink/Math Operations/Product', [modelName '/v*cos'], 'Position', [320 55 360 85]);
  add_block('simulink/Math Operations/Product', [modelName '/v*sin'], 'Position', [320 125 360 155]);

  add_block('simulink/Continuous/Integrator', [modelName '/x'], 'Position', [420 55 450 85]);
  add_block('simulink/Continuous/Integrator', [modelName '/y'], 'Position', [420 125 450 155]);
  add_block('simulink/Continuous/Integrator', [modelName '/theta'], 'Position', [220 200 250 230]);

  add_block('simulink/Sinks/To Workspace', [modelName '/x_out'], 'VariableName', 'x_out', 'SaveFormat', 'Array', 'Position', [520 55 600 85]);
  add_block('simulink/Sinks/To Workspace', [modelName '/y_out'], 'VariableName', 'y_out', 'SaveFormat', 'Array', 'Position', [520 125 600 155]);
  add_block('simulink/Sinks/To Workspace', [modelName '/theta_out'], 'VariableName', 'theta_out', 'SaveFormat', 'Array', 'Position', [320 200 410 230]);

  add_line(modelName, 'theta/1', 'cos/1'); add_line(modelName, 'theta/1', 'sin/1');
  add_line(modelName, 'Vin/1', 'v*cos/1'); add_line(modelName, 'cos/1', 'v*cos/2');
  add_line(modelName, 'Vin/1', 'v*sin/1'); add_line(modelName, 'sin/1', 'v*sin/2');
  add_line(modelName, 'v*cos/1', 'x/1'); add_line(modelName, 'v*sin/1', 'y/1');
  add_line(modelName, 'Win/1', 'theta/1');
  add_line(modelName, 'x/1', 'x_out/1'); add_line(modelName, 'y/1', 'y_out/1'); add_line(modelName, 'theta/1', 'theta_out/1');

  save_system(modelName);
end
