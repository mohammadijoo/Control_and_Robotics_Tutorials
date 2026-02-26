% Plant G(s) = K / (s (s + 1))
K = 5;
num = K;
den = [1 1 0];   % s^2 + s = s (s + 1)
G = tf(num, den);

% Unity feedback closed loop
sys_cl = feedback(G, 1);

% Time vector
t = 0:0.01:10;

% STEP input
[r_step, ~] = gensig("step", 10, 10, 0.01); %#ok<ASGLU>
r_step = ones(size(t));
[y_step, ~] = step(sys_cl, t);
e_step = r_step - y_step;

% RAMP input: r(t) = t
r_ramp = t;
[y_ramp, ~] = lsim(sys_cl, r_ramp, t);
e_ramp = r_ramp - y_ramp;

figure;
plot(t, e_step, t, e_ramp);
grid on;
legend("e\_step(t)", "e\_ramp(t)");
xlabel("t [s]");
ylabel("error");
title("Error signals for step and ramp references");

% In Simulink, the same system can be built by connecting:
% - Step or Ramp block (reference)
% - Sum block (for e = r - y)
% - Controller block (here unity gain)
% - Transfer Fcn block for G(s)
% - Scope blocks to observe y(t) and e(t).
% Robotics System Toolbox allows connecting these models to
% ROS topics or to rigid-body robot models.
