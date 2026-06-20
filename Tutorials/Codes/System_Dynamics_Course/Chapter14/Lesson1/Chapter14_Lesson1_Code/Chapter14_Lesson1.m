% System Dynamics — Chapter 14 (Nonlinear System Dynamics)
% Lesson 1: Sources and Types of Nonlinearities in Engineering Systems
%
% Nonlinear mass–spring–damper simulation using ode45.
% Saves trajectory to CSV for cross-language comparison.

clear; clc;

p.m = 1.0;
p.c = 0.4;
p.k = 4.0;
p.k3 = 8.0;
p.Fc = 0.8;
p.vs = 0.02;
p.b = 1.0;
p.umax = 1.5;

tspan = [0 20];
x0 = [0.4; 0.0];  % [position; velocity]

opts = odeset('RelTol',1e-7,'AbsTol',1e-9,'MaxStep',0.01);
[t, X] = ode45(@(t,x) dyn(t,x,p), tspan, x0, opts);

% Export to CSV
T = table(t, X(:,1), X(:,2), 'VariableNames', {'t','x','xdot'});
writetable(T, 'Chapter14_Lesson1_trace_matlab.csv');

% Plot
figure; plot(t, X(:,1)); grid on;
xlabel('t [s]'); ylabel('x [m]'); title('Nonlinear position');

figure; plot(t, X(:,2)); grid on;
xlabel('t [s]'); ylabel('xdot [m/s]'); title('Nonlinear velocity');

disp('Wrote: Chapter14_Lesson1_trace_matlab.csv');

function dx = dyn(t, x, p)
    pos = x(1);
    vel = x(2);

    u = input_u(t);
    u_sat = sat(u, p.umax);

    spring = p.k*pos + p.k3*pos^3;
    fric = p.c*vel + p.Fc*tanh(vel/p.vs);

    acc = (p.b*u_sat - spring - fric) / p.m;

    dx = [vel; acc];
end

function u = input_u(t)
    u = 1.2*sin(1.0*t) + 0.3*sin(3.0*t);
end

function y = sat(u, umax)
    y = min(max(u, -umax), umax);
end
