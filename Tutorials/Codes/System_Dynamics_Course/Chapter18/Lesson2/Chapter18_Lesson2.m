% Chapter18_Lesson2.m
% Lagrangian modeling and simulation of a cart-pendulum system
% Generalized coordinates: q = [x; theta]
% Uses ode45 and exports simulation results.

clear; clc;

p.M = 1.0;
p.m = 0.25;
p.l = 0.5;
p.J = 0.02;
p.g = 9.81;
p.k = 8.0;
p.c = 0.35;
p.b = 0.05;

x0 = [0.05; 0.35; 0.0; 0.0]; % [x; theta; xdot; thetadot]

tspan = [0 10];
opts = odeset('RelTol',1e-8,'AbsTol',1e-9);
[t, X] = ode45(@(t,x) cartPendulumRHS(t, x, p), tspan, x0, opts);

E = zeros(length(t),1);
for i = 1:length(t)
    E(i) = totalEnergy(X(i,:).', p);
end

figure;
plot(t, X(:,1), 'LineWidth', 1.2); hold on;
plot(t, X(:,2), 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('States');
legend('x (m)','theta (rad)');
title('Cart-Pendulum Generalized Coordinates');

figure;
plot(t, E, 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Energy (J)');
title('Total Mechanical Energy');

T = table(t, X(:,1), X(:,2), X(:,3), X(:,4), E, ...
    'VariableNames', {'t','x','theta','xdot','thetadot','energy'});
writetable(T, 'Chapter18_Lesson2_matlab_output.csv');

disp('Saved Chapter18_Lesson2_matlab_output.csv');

% Optional symbolic verification (requires Symbolic Math Toolbox)
% syms x(t) th(t) M m l J g k c b u real
% xd = diff(x,t); thd = diff(th,t);
% T = 1/2*(M+m)*xd^2 + m*l*cos(th)*xd*thd + 1/2*(J+m*l^2)*thd^2;
% V = 1/2*k*x^2 + m*g*l*(1-cos(th));
% D = 1/2*c*xd^2 + 1/2*b*thd^2;
% L = T - V;
% Eqx  = simplify(diff(diff(L, xd), t) - diff(L, x) + diff(D, xd) - u);
% Eqth = simplify(diff(diff(L, thd), t) - diff(L, th) + diff(D, thd));
% pretty(Eqx), pretty(Eqth)

function dx = cartPendulumRHS(t, x, p)
    q1 = x(1); th = x(2); q1d = x(3); thd = x(4);

    if t >= 0.5 && t <= 2.5
        u = 2.0;
    else
        u = 0.0;
    end

    M11 = p.M + p.m;
    M12 = p.m * p.l * cos(th);
    M21 = M12;
    M22 = p.J + p.m * p.l^2;

    rhs1 = u - p.c*q1d - p.k*q1 + p.m*p.l*sin(th)*thd^2;
    rhs2 = -p.b*thd - p.m*p.g*p.l*sin(th);

    detM = M11*M22 - M12*M21;
    q1dd = (rhs1*M22 - rhs2*M12) / detM;
    thdd = (M11*rhs2 - M21*rhs1) / detM;

    dx = [q1d; thd; q1dd; thdd];
end

function E = totalEnergy(x, p)
    q1 = x(1); th = x(2); q1d = x(3); thd = x(4);
    T = 0.5*(p.M + p.m)*q1d^2 + p.m*p.l*cos(th)*q1d*thd + 0.5*(p.J + p.m*p.l^2)*thd^2;
    V = 0.5*p.k*q1^2 + p.m*p.g*p.l*(1 - cos(th));
    E = T + V;
end
