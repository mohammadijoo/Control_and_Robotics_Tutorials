% Chapter7_Lesson4.m
% Kalman-Filter Localization for AMR — Lesson 4
% EKF tuning via innovation statistics (NIS) for a unicycle + GPS example.
%
% This script is self-contained and uses only base MATLAB.
% Optional: build a simple Simulink shell (requires Simulink) by calling:
%   build_simulink_shell_Chapter7_Lesson4();

function Chapter7_Lesson4()
  dt = 0.1;
  [xTrue, uMeas, zGps] = simulate_data(60.0, dt, 2);

  % Initial guesses (deliberately imperfect)
  p.sigma_v = 0.03;
  p.sigma_w = 0.01;
  p.sigma_gps = 0.3;
  p.q_scale = 1.0;
  p.r_scale = 1.0;

  for it = 1:4
    [nisList, neesList] = run_filter_stats(xTrue, uMeas, zGps, dt, p);

    nisMean = mean(nisList);
    neesMean = mean(neesList);

    fprintf('Iter %d: r_scale=%.3f, q_scale=%.3f\\n', it, p.r_scale, p.q_scale);
    fprintf('  mean NIS=%.3f (target ~ 2), mean NEES=%.3f (target ~ 3)\\n', nisMean, neesMean);

    % Heuristic update for R using NIS
    gain = min(5.0, max(0.2, nisMean/2.0));
    p.r_scale = p.r_scale * gain;
  end

  disp('Final tuned parameters:');
  disp(p);
end

function a = wrap_angle(a)
  a = mod(a + pi, 2*pi) - pi;
end

function x2 = f_unicycle(x, u, dt)
  px = x(1); py = x(2); th = x(3);
  v = u(1);  w = u(2);
  x2 = zeros(3,1);
  x2(1) = px + v*dt*cos(th);
  x2(2) = py + v*dt*sin(th);
  x2(3) = wrap_angle(th + w*dt);
end

function F = jacobian_F(x, u, dt)
  th = x(3); v = u(1);
  F = eye(3);
  F(1,3) = -v*dt*sin(th);
  F(2,3) =  v*dt*cos(th);
end

function L = jacobian_L(x, dt)
  th = x(3);
  L = zeros(3,2);
  L(1,1) = dt*cos(th);
  L(2,1) = dt*sin(th);
  L(3,2) = dt;
end

function Q = make_Q(x, u, dt, p)
  L = jacobian_L(x, dt);
  M = diag([p.sigma_v^2, p.sigma_w^2]);
  Q = p.q_scale * (L*M*L');
  %#ok<NASGU>
end

function zhat = h_gps(x)
  zhat = x(1:2);
end

function H = jacobian_H_gps()
  H = zeros(2,3);
  H(1,1) = 1;
  H(2,2) = 1;
end

function R = make_R_gps(p)
  R = p.r_scale * diag([p.sigma_gps^2, p.sigma_gps^2]);
end

function [nisList, neesList] = run_filter_stats(xTrue, uMeas, zGps, dt, p)
  N = size(xTrue,1);
  x = [0;0;0];
  P = diag([1,1,(10*pi/180)^2]);

  H = jacobian_H_gps();
  nisList = zeros(N,1);
  neesList = zeros(N,1);

  for k = 1:N
    % Predict
    xPred = f_unicycle(x, uMeas(k,:)', dt);
    F = jacobian_F(x, uMeas(k,:)', dt);
    Q = make_Q(x, uMeas(k,:)', dt, p);
    PPred = F*P*F' + Q;

    % Update
    y = zGps(k,:)' - h_gps(xPred);
    S = H*PPred*H' + make_R_gps(p);
    K = PPred*H' / S;
    x = xPred + K*y;
    x(3) = wrap_angle(x(3));
    P = (eye(3) - K*H)*PPred;

    nisList(k) = y' / S * y;
    e = x - xTrue(k,:)';
    e(3) = wrap_angle(e(3));
    neesList(k) = e' / P * e;
  end
end

function [xTrue, uMeas, zGps] = simulate_data(T, dt, seed)
  rng(seed);
  N = floor(T/dt);
  xTrue = zeros(N,3);
  uTrue = zeros(N,2);

  for k = 1:N
    t = (k-1)*dt;
    uTrue(k,1) = 1.0 + 0.2*sin(0.2*t);
    uTrue(k,2) = 0.2*sin(0.1*t);
    if k > 1
      xTrue(k,:) = f_unicycle(xTrue(k-1,:)', uTrue(k-1,:)', dt)';
    end
  end

  sigma_v_meas = 0.08;
  sigma_w_meas = 0.04;
  sigma_gps_meas = 0.6;

  uMeas = uTrue + randn(N,2).* [sigma_v_meas, sigma_w_meas];
  zGps  = xTrue(:,1:2) + randn(N,2).* sigma_gps_meas;
end

function build_simulink_shell_Chapter7_Lesson4()
  % Creates a minimal Simulink shell illustrating where EKF prediction/update
  % blocks go. You still need to implement the EKF equations inside a
  % MATLAB Function block or use a suitable toolbox implementation.

  mdl = 'Chapter7_Lesson4_Simulink';
  if bdIsLoaded(mdl), close_system(mdl, 0); end
  new_system(mdl); open_system(mdl);

  add_block('simulink/Sources/From Workspace', [mdl '/uMeas']);
  add_block('simulink/Sources/From Workspace', [mdl '/zGps']);
  add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/EKF_Core']);
  add_block('simulink/Sinks/To Workspace', [mdl '/xEst']);

  set_param([mdl '/uMeas'], 'VariableName', 'uMeas');
  set_param([mdl '/zGps'], 'VariableName', 'zGps');
  set_param([mdl '/xEst'], 'VariableName', 'xEst');

  add_line(mdl, 'uMeas/1', 'EKF_Core/1');
  add_line(mdl, 'zGps/1', 'EKF_Core/2');
  add_line(mdl, 'EKF_Core/1', 'xEst/1');

  set_param(mdl, 'StopTime', '60');
  save_system(mdl);
  disp(['Created Simulink shell: ' mdl '.slx']);
end
