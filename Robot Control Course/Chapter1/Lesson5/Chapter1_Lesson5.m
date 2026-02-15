
zeta = 0.7;
wn   = 8.0;

num = [wn^2];
den = [1, 2*zeta*wn, wn^2];
sys = tf(num, den);

info = stepinfo(sys);  % contains RiseTime, SettlingTime, Overshoot, etc.
disp(info);

% Compute steady-state error and energy-based metrics numerically
t = 0:1e-3:5;
[y, t] = step(sys, t);
r = ones(size(t));
e = r - y;

ess = e(end);
J_ise = trapz(t, e.^2);

% Simple PD control energy for unit inertia
Kp = wn^2;
Kd = 2*zeta*wn;
de = gradient(e, t);
u = Kp*e + Kd*de;
J_u = trapz(t, u.^2);

fprintf('ess = %g\n', ess);
fprintf('J_ise = %g\n', J_ise);
fprintf('J_u = %g\n', J_u);

% In Simulink, you would build a block diagram:
% - Second-order plant G(s)
% - Step input
% - Scope blocks to log y(t), e(t), u(t)
% and then post-process using the same formulas.
