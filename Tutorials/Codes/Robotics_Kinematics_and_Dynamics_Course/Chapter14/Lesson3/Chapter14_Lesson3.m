function dx = flex_joint_dynamics(t, x, p)
% x = [q; qdot; th; thdot]

q     = x(1);
qdot  = x(2);
th    = x(3);
thdot = x(4);

spring = p.k * (th - q);
damper = p.d * (thdot - qdot);
tau_m  = p.tau0;  % step input

qddot  = (spring + damper - p.m * p.g * p.ell * sin(q) - p.b_l * qdot) / p.J_l;
thddot = (tau_m - spring - damper - p.b_m * thdot) / p.J_m;

dx = [qdot; qddot; thdot; thddot];
end

% Script to simulate
p.J_l = 0.5;  p.J_m = 0.05;
p.b_l = 0.02; p.b_m = 0.01;
p.k   = 150;  p.d   = 0.5;
p.m   = 2.0;  p.ell = 0.4; p.g = 9.81;
p.tau0 = 1.0;

x0 = [0; 0; 0; 0];
[t, x] = ode45(@(t,x) flex_joint_dynamics(t,x,p), [0 2], x0);

q  = x(:,1);
th = x(:,3);
plot(t, q, t, th, '--'); legend('link q', 'motor theta');
xlabel('time [s]'); ylabel('angle [rad]'); grid on;
      
