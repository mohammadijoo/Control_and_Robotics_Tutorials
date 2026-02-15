
% Parameters
J = 0.05;
b = 0.01;
m = 1.0;
ell = 0.5;
g = 9.81;

qd = deg2rad(45);  % desired angle

wn = 8.0;
zeta = 0.7;

Kp = J * wn^2;
Kd = 2 * J * zeta * wn - b;

% continuous dynamics with controller
dyn = @(t, x) [
    x(2);  % x(1) = q, x(2) = dq
    ( ...
      Kp * (qd - x(1)) + ...
      Kd * (0 - x(2)) + ...
      m * g * ell * sin(x(1)) - ...
      b * x(2) - ...
      m * g * ell * sin(x(1)) ...
    ) / J  % J ddq + b dq + g(q) = tau
];

x0 = [0; 0];
tspan = [0 2];
[t, x] = ode45(dyn, tspan, x0);

q = x(:,1);
dq = x(:,2);

figure;
subplot(3,1,1);
plot(t, q); hold on;
yline(qd, "--");
ylabel("q [rad]");
legend("q", "q_d");

subplot(3,1,2);
plot(t, dq);
ylabel("dq [rad/s]");

% compute torque trajectory
tau = Kp * (qd - q) + Kd * (0 - dq) + m * g * ell * sin(q);
subplot(3,1,3);
plot(t, tau);
ylabel("tau [N m]");
xlabel("time [s]");
