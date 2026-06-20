% Chapter4_Lesson3.m
% Simple dynamic model for a ground robot: velocity-lag unicycle.
% Includes: (1) ODE45 simulation, (2) exact discrete-time update for first-order lag,
% (3) programmatic Simulink model builder (optional).
%
% Requires: MATLAB base. For the Simulink model builder, Simulink must be installed.

clear; clc;

% Parameters
T_v = 0.30;   % [s]
T_w = 0.25;   % [s]

% Step commands
v_ref = @(t) (t >= 0.5) * 1.0;
w_ref = @(t) (t >= 2.0) * 0.7;

% ODE definition: x = [px; py; theta; v; w]
f = @(t,x) [
  x(4)*cos(x(3));
  x(4)*sin(x(3));
  x(5);
  (v_ref(t) - x(4))/T_v;
  (w_ref(t) - x(5))/T_w
];

tspan = [0 8];
x0 = [0;0;0;0;0];

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,X] = ode45(f, tspan, x0, opts);

fprintf('Final pose: px=%.3f m, py=%.3f m, theta=%.3f rad\n', X(end,1), X(end,2), X(end,3));
fprintf('Final body velocities: v=%.3f m/s, w=%.3f rad/s\n', X(end,4), X(end,5));

% Plot (optional)
figure; plot(X(:,1), X(:,2), 'LineWidth', 1.5); axis equal; grid on;
xlabel('px [m]'); ylabel('py [m]'); title('Trajectory (lag-unicycle)');

figure; plot(t, X(:,4), 'LineWidth', 1.5); hold on; plot(t, X(:,5), 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('v, w'); legend('v','w'); title('Body velocities');

% Exact discrete-time update for zdot = (z_ref - z)/T with sample time dt
exactUpdate = @(z, zref, T, dt) exp(-dt/T)*z + (1 - exp(-dt/T))*zref;

% Demonstration of discrete update for v only
dt = 0.05;
v = 0;
for k = 1:round(8/dt)
    tk = (k-1)*dt;
    v = exactUpdate(v, v_ref(tk), T_v, dt);
end
fprintf('Discrete-time v at t=8s (piecewise-constant): %.3f\n', v);

% --- Optional: build a Simulink model programmatically ---
% The model implements:
%   vdot = (v_ref - v)/T_v,  wdot = (w_ref - w)/T_w
%   xdot = v cos(theta), ydot = v sin(theta), thetadot = w

buildSimulink = false;  % set true to build model
if buildSimulink
    mdl = 'Chapter4_Lesson3_Simulink';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    % Add blocks (Integrator, Trig, Product, Sum, Gain, Step)
    add_block('simulink/Sources/Step', [mdl '/v_ref'], 'Position',[50 60 80 90], ...
              'Time','0.5','Before','0','After','1');
    add_block('simulink/Sources/Step', [mdl '/w_ref'], 'Position',[50 140 80 170], ...
              'Time','2','Before','0','After','0.7');

    add_block('simulink/Math Operations/Sum', [mdl '/Sum_v'], 'Inputs','+-', 'Position',[140 60 160 90]);
    add_block('simulink/Math Operations/Gain', [mdl '/Gain_v'], 'Gain',sprintf('1/%g',T_v), 'Position',[190 60 240 90]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_v'], 'Position',[270 60 300 90]);

    add_block('simulink/Math Operations/Sum', [mdl '/Sum_w'], 'Inputs','+-', 'Position',[140 140 160 170]);
    add_block('simulink/Math Operations/Gain', [mdl '/Gain_w'], 'Gain',sprintf('1/%g',T_w), 'Position',[190 140 240 170]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_w'], 'Position',[270 140 300 170]);

    add_block('simulink/Continuous/Integrator', [mdl '/Int_theta'], 'Position',[360 140 390 170]);

    add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cos'], 'Operator','cos', 'Position',[420 60 460 90]);
    add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sin'], 'Operator','sin', 'Position',[420 100 460 130]);

    add_block('simulink/Math Operations/Product', [mdl '/Prod_xdot'], 'Position',[500 60 530 90]);
    add_block('simulink/Math Operations/Product', [mdl '/Prod_ydot'], 'Position',[500 100 530 130]);

    add_block('simulink/Continuous/Integrator', [mdl '/Int_x'], 'Position',[560 60 590 90]);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y'], 'Position',[560 100 590 130]);

    % Wiring
    add_line(mdl,'v_ref/1','Sum_v/1');
    add_line(mdl,'Int_v/1','Sum_v/2');
    add_line(mdl,'Sum_v/1','Gain_v/1');
    add_line(mdl,'Gain_v/1','Int_v/1');

    add_line(mdl,'w_ref/1','Sum_w/1');
    add_line(mdl,'Int_w/1','Sum_w/2');
    add_line(mdl,'Sum_w/1','Gain_w/1');
    add_line(mdl,'Gain_w/1','Int_w/1');

    add_line(mdl,'Int_w/1','Int_theta/1');

    add_line(mdl,'Int_theta/1','cos/1');
    add_line(mdl,'Int_theta/1','sin/1');

    add_line(mdl,'Int_v/1','Prod_xdot/1');
    add_line(mdl,'cos/1','Prod_xdot/2');
    add_line(mdl,'Int_v/1','Prod_ydot/1');
    add_line(mdl,'sin/1','Prod_ydot/2');

    add_line(mdl,'Prod_xdot/1','Int_x/1');
    add_line(mdl,'Prod_ydot/1','Int_y/1');

    set_param(mdl,'StopTime','8');
    save_system(mdl);
    fprintf('Built and saved Simulink model: %s.slx\n', mdl);
end
