% Parameters
a = 1.0;
b = 1.0;
k = 5.0;
r = 1.0;

dt = 0.001;
t_final = 5.0;
t = 0:dt:t_final;
n_steps = numel(t);

x = zeros(size(t));
u = zeros(size(t));

for i = 1:n_steps-1
    e = r - x(i);
    u(i) = k * e;
    dx = -(a + b * k) * x(i) + b * k * r;
    x(i+1) = x(i) + dt * dx;
end
u(end) = k * (r - x(end));

x_ss = x(end);
e_ss = r - x_ss;

fprintf("Approx steady-state output y_ss = %f\n", x_ss);
fprintf("Approx steady-state error e_ss = %f\n", e_ss);
fprintf("Peak control effort max|u|    = %f\n", max(abs(u)));

% In Simulink, the same closed loop can be built using a Sum block for e(t),
% a Gain block for k, another Sum block for plant dynamics, and an Integrator
% block for the state x(t). MATLAB's Robotics System Toolbox can later be
s% used to connect such controllers to robot models.
