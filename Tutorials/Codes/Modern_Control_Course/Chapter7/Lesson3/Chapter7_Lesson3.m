% Chapter7_Lesson3.m
% Modern Control — Chapter 7 (Solutions of LTI State Equations), Lesson 3
% Solutions for Constant, Step, and Polynomial Inputs
%
% This script demonstrates closed-form state responses using matrix exponentials
% and phi-functions for:
%   1) constant input u(t)=u0
%   2) step input u(t)=u0 * 1(t-ts)
%   3) polynomial input u(t)=u0 + u1 t
%
% It also verifies results by numerical integration via ode45.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
x0 = [1; 0];

u0 = 2;
u1 = 0.5;

ts = 1.5;

tgrid = linspace(0, 5, 251);

% --- phi-function via truncated series: phi_m(Z) = sum_{j=0}^∞ Z^j/(j+m)! ---
phi_series = @(Z,m,terms) local_phi_series(Z,m,terms);

x_const = zeros(2, numel(tgrid));
x_step  = zeros(2, numel(tgrid));
x_poly  = zeros(2, numel(tgrid));

for i = 1:numel(tgrid)
    t = tgrid(i);

    % Constant input
    At = A*t;
    E = expm(At);
    Phi1 = phi_series(At, 1, 40);
    x_const(:,i) = E*x0 + t * (Phi1*(B*u0));

    % Step input
    if t < ts
        x_step(:,i) = expm(A*t)*x0;
    else
        dt = t - ts;
        Phi1_dt = phi_series(A*dt, 1, 40);
        x_step(:,i) = expm(A*t)*x0 + dt * (Phi1_dt*(B*u0));
    end

    % Polynomial input u(t) = u0 + u1 t
    % x(t) = e^{At}x0 + t*phi_1(At)B u0 + t^2 * 1! * phi_2(At)B u1
    Phi2 = phi_series(At, 2, 45);
    x_poly(:,i) = E*x0 + t * (Phi1*(B*u0)) + (t^2) * (Phi2*(B*u1));
end

% --- Numerical verification with ode45 ---
opts = odeset('RelTol',1e-10,'AbsTol',1e-12);

% 1) constant input
ode_const = @(t,x) A*x + B*u0;
[tn1, xn1] = ode45(ode_const, [tgrid(1) tgrid(end)], x0, opts);
xn1i = interp1(tn1, xn1, tgrid);

err1 = max(vecnorm((x_const' - xn1i), 2, 2));
fprintf('Constant input: max ||x_closed - x_numeric|| = %.3e\n', err1);

% 2) step input
ode_step = @(t,x) A*x + B*(u0*(t>=ts));
[tn2, xn2] = ode45(ode_step, [tgrid(1) tgrid(end)], x0, opts);
xn2i = interp1(tn2, xn2, tgrid);

err2 = max(vecnorm((x_step' - xn2i), 2, 2));
fprintf('Step input: max ||x_closed - x_numeric|| = %.3e\n', err2);

% 3) polynomial input
ode_poly = @(t,x) A*x + B*(u0 + u1*t);
[tn3, xn3] = ode45(ode_poly, [tgrid(1) tgrid(end)], x0, opts);
xn3i = interp1(tn3, xn3, tgrid);

err3 = max(vecnorm((x_poly' - xn3i), 2, 2));
fprintf('Polynomial input: max ||x_closed - x_numeric|| = %.3e\n', err3);

% Optional plot
figure;
plot(tgrid, x_const(1,:), tgrid, x_step(1,:), tgrid, x_poly(1,:));
xlabel('t'); ylabel('x_1(t)');
legend('constant', 'step', 'polynomial');
grid on;

% --- Simulink (optional): programmatic model creation for step input ---
% To keep the lesson self-contained, we show how to generate a simple Simulink
% model with a State-Space block and a Step block.
%
% Uncomment to create:
% local_build_simulink_model(A,B,[1 0],0,ts,u0);

% --------- local functions ---------
function Phi = local_phi_series(Z, m, terms)
    n = size(Z,1);
    Phi = eye(n)/factorial(m);
    Zpow = eye(n);
    for j = 1:terms-1
        Zpow = Zpow*Z;
        Phi = Phi + Zpow/factorial(j+m);
    end
end

function local_build_simulink_model(A,B,C,D,ts,amp)
    mdl = 'Chapter7_Lesson3_Simulink';
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Sources/Step',[mdl '/Step'], 'Time', num2str(ts), ...
              'Before', '0', 'After', num2str(amp));
    add_block('simulink/Continuous/State-Space',[mdl '/StateSpace']);
    set_param([mdl '/StateSpace'], 'A', mat2str(A), 'B', mat2str(B), ...
              'C', mat2str(C), 'D', mat2str(D));
    add_block('simulink/Sinks/Scope',[mdl '/Scope']);

    set_param([mdl '/StateSpace'], 'Position', [200 80 350 150]);
    set_param([mdl '/Step'], 'Position', [50 90 100 120]);
    set_param([mdl '/Scope'], 'Position', [430 80 460 110]);

    add_line(mdl,'Step/1','StateSpace/1');
    add_line(mdl,'StateSpace/1','Scope/1');

    set_param(mdl,'StopTime','5');
    save_system(mdl);
end
