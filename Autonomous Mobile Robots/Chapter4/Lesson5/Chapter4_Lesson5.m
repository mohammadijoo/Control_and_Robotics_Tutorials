% Chapter4_Lesson5.m
% Parameter Effects on Real Navigation (Mobile Robot Dynamics - Applied)
%
% MATLAB/Simulink-oriented implementation:
% 1) ODE simulation of a differential-drive robot with simple v/omega dynamics.
% 2) Parameter sweep showing RMS deviation when plant parameters differ from nominal.
% 3) (Optional) A small script section that programmatically constructs an equivalent
%    Simulink model for the v/omega channels (kinematics can be added similarly).
%
% Recommended toolboxes:
% - Robotics System Toolbox (for mobile robotics utilities, ROS interfaces)
% - Simulink (for block-diagram modeling)
%
% Run:
%   Chapter4_Lesson5

clear; clc;

% -----------------------------
% Parameters (nominal controller model)
% -----------------------------
p_nom.m  = 25.0;
p_nom.Iz = 2.0;
p_nom.b  = 0.45;
p_nom.r  = 0.10;
p_nom.mu = 0.8;
p_nom.cv = 0.4;
p_nom.cw = 0.6;
p_nom.g  = 9.81;

% Simulation horizon
T  = 35.0;
dt = 0.01;
ts = 0:dt:T;

% Baseline nominal trajectory
[x0, ~] = simulatePlant(p_nom, p_nom, ts);

% Sweep list (true plant varies; controller stays nominal)
sweep = {
  'mass_m',      [15.0 25.0 40.0], 'kg';
  'yaw_Iz',      [1.2  2.0  3.5 ], 'kg*m^2';
  'wheel_r',     [0.095 0.10 0.105], 'm';
  'wheelbase_b', [0.40 0.45 0.52], 'm';
  'friction_mu', [0.5  0.8  1.0 ], '-'
};

fprintf('Parameter sweep results (true plant varies; controller stays nominal):\n');
for i=1:size(sweep,1)
  name = sweep{i,1};
  vals = sweep{i,2};
  unit = sweep{i,3};
  for v = vals
    p_true = p_nom;
    switch name
      case 'mass_m',      p_true.m  = v;
      case 'yaw_Iz',      p_true.Iz = v;
      case 'wheel_r',     p_true.r  = v;
      case 'wheelbase_b', p_true.b  = v;
      case 'friction_mu', p_true.mu = v;
    end
    [x, ~] = simulatePlant(p_true, p_nom, ts);
    pos_err = sqrt((x(:,1)-x0(:,1)).^2 + (x(:,2)-x0(:,2)).^2);
    th_err  = unwrap(x(:,3)) - unwrap(x0(:,3));

    rms_pos = sqrt(mean(pos_err.^2));
    rms_th  = sqrt(mean(th_err.^2));
    fprintf('%s=%7.3f %7s | pos_rms=%6.3f m, theta_rms=%6.3f rad\n', name, v, unit, rms_pos, rms_th);
  end
end

% -----------------------------
% Optional: Programmatic Simulink model (v and omega channels only)
% -----------------------------
%{
mdl = 'Chapter4_Lesson5_Simulink';
if bdIsLoaded(mdl); close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

% Blocks: Integrator for v, Integrator for omega, Gains, Sum blocks
add_block('simulink/Continuous/Integrator', [mdl '/Int_v'], 'Position', [180 80 210 110]);
add_block('simulink/Continuous/Integrator', [mdl '/Int_om'], 'Position', [180 160 210 190]);

add_block('simulink/Math Operations/Gain', [mdl '/Gain_1_m'], 'Gain', num2str(1/p_nom.m), 'Position', [110 70 150 110]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain_1_Iz'], 'Gain', num2str(1/p_nom.Iz), 'Position', [110 150 150 190]);

add_block('simulink/Math Operations/Gain', [mdl '/Gain_cv'], 'Gain', num2str(-p_nom.cv), 'Position', [260 70 300 110]);
add_block('simulink/Math Operations/Gain', [mdl '/Gain_cw'], 'Gain', num2str(-p_nom.cw), 'Position', [260 150 300 190]);

add_block('simulink/Math Operations/Sum', [mdl '/Sum_vdot'], 'Inputs', '++', 'Position', [320 80 350 110]);
add_block('simulink/Math Operations/Sum', [mdl '/Sum_omdot'], 'Inputs', '++', 'Position', [320 160 350 190]);

add_block('simulink/Sources/In1', [mdl '/F_in'], 'Position', [30 80 60 100]);
add_block('simulink/Sources/In1', [mdl '/Mz_in'], 'Position', [30 160 60 180]);

% Connect lines: vdot = (1/m)*F + (-cv)*v
add_line(mdl, 'F_in/1', 'Gain_1_m/1');
add_line(mdl, 'Gain_1_m/1', 'Sum_vdot/1');
add_line(mdl, 'Int_v/1', 'Gain_cv/1');
add_line(mdl, 'Gain_cv/1', 'Sum_vdot/2');
add_line(mdl, 'Sum_vdot/1', 'Int_v/1');

% omega_dot = (1/Iz)*Mz + (-cw)*omega
add_line(mdl, 'Mz_in/1', 'Gain_1_Iz/1');
add_line(mdl, 'Gain_1_Iz/1', 'Sum_omdot/1');
add_line(mdl, 'Int_om/1', 'Gain_cw/1');
add_line(mdl, 'Gain_cw/1', 'Sum_omdot/2');
add_line(mdl, 'Sum_omdot/1', 'Int_om/1');

save_system(mdl);
disp(['Created Simulink model: ' mdl]);
%}

% -----------------------------
% Local functions
% -----------------------------
function [xs, refs] = simulatePlant(p_true, p_nom, ts)
  % State x = [px, py, theta, v, omega]
  x = zeros(5,1);
  xs = zeros(numel(ts), 5);
  refs = zeros(numel(ts), 2);

  % Controller integrators
  iv = 0; iw = 0;
  kvp=180.0; kvi=35.0; kwp=18.0; kwi=4.0;
  tau_max = 35.0;

  for k=1:numel(ts)
    t = ts(k);
    [v_ref, om_ref] = referenceCmd(t);
    refs(k,:) = [v_ref, om_ref];

    % Control (computed with nominal parameters)
    ev = v_ref - x(4);
    ew = om_ref - x(5);
    iv = iv + ev*(ts(2)-ts(1));
    iw = iw + ew*(ts(2)-ts(1));

    F_cmd = p_nom.m  * (kvp*ev + kvi*iv) + p_nom.m  * p_nom.cv * x(4);
    M_cmd = p_nom.Iz * (kwp*ew + kwi*iw) + p_nom.Iz * p_nom.cw * x(5);

    % Map to wheel torques with nominal geometry
    tauR = 0.5 * p_nom.r * (F_cmd + 2*M_cmd/p_nom.b);
    tauL = 0.5 * p_nom.r * (F_cmd - 2*M_cmd/p_nom.b);
    tauR = max(min(tauR, tau_max), -tau_max);
    tauL = max(min(tauL, tau_max), -tau_max);

    % One RK4 step (true plant)
    dt = ts(2)-ts(1);
    x = rk4(@(xx) f(xx, tauL, tauR, p_true), x, dt);

    xs(k,:) = x.';
  end
end

function dx = f(x, tauL, tauR, p)
  px=x(1); py=x(2); th=x(3); v=x(4); om=x(5);

  F  = (tauR + tauL) / p.r;
  Mz = (p.b / (2*p.r)) * (tauR - tauL);

  Fmax = p.mu * p.m * p.g;
  F = max(min(F, Fmax), -Fmax);

  vdot  = (1/p.m)*F  - p.cv*v;
  omdot = (1/p.Iz)*Mz - p.cw*om;

  pxdot = v*cos(th);
  pydot = v*sin(th);
  thdot = om;

  dx = [pxdot; pydot; thdot; vdot; omdot];
end

function xnext = rk4(fun, x, dt)
  k1 = fun(x);
  k2 = fun(x + 0.5*dt*k1);
  k3 = fun(x + 0.5*dt*k2);
  k4 = fun(x + dt*k3);
  xnext = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function [v_ref, om_ref] = referenceCmd(t)
  v_ref  = 0.9 + 0.2*sin(0.2*t);
  om_ref = 0.35 + 0.15*sin(0.17*t + 0.7);
end
