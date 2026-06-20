% Chapter15_Lesson2.m
% Runge-Kutta Methods and Step Size Selection
% MATLAB script: RK2, RK4, adaptive RK4 (step-doubling), and ode45 comparison.

clear; clc;

f = @(t,y) -2*y + sin(t);
yExact = @(t) (6/5)*exp(-2*t) + (2*sin(t) - cos(t))/5;

t0 = 0; tf = 10; y0 = 1;

% Fixed-step RK2 and RK4
h = 0.1;
[t2, y2] = integrateFixed(@rk2midpointStep, f, t0, tf, y0, h);
[t4, y4] = integrateFixed(@rk4Step,       f, t0, tf, y0, h);

% Adaptive RK4 with step-doubling
res = integrateAdaptiveRK4StepDoubling(f, t0, tf, y0, 0.2, 1e-8, 1e-6, 1e-8, 0.5, 0.9);

% MATLAB built-in adaptive RK pair (Dormand-Prince)
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t45, y45] = ode45(f, [t0 tf], y0, opts);

fprintf('Chapter15_Lesson2.m\n');
fprintf('RK2 midpoint (h=%.3f), max error = %.6e\n', h, max(abs(y2 - yExact(t2))));
fprintf('RK4         (h=%.3f), max error = %.6e\n', h, max(abs(y4 - yExact(t4))));
fprintf('Adaptive RK4 step-doubling: accepted=%d, rejected=%d, max error=%.6e\n', ...
    res.accepted, res.rejected, max(abs(res.y - yExact(res.t))));
fprintf('ode45 internal steps = %d, max error = %.6e\n', length(t45)-1, max(abs(y45 - yExact(t45))));

figure;
tt = linspace(t0, tf, 1000);
plot(tt, yExact(tt), 'LineWidth', 1.5); hold on;
plot(t2, y2, '.');
plot(t4, y4, '-');
plot(res.t, res.y, 'o', 'MarkerSize', 3);
legend('Exact','RK2','RK4','Adaptive RK4');
xlabel('t'); ylabel('y(t)'); grid on;
title('Chapter15\_Lesson2: RK Methods');

figure;
plot(res.t(2:end), res.hHistory, '-o', 'MarkerSize', 3);
xlabel('t'); ylabel('Accepted step size h'); grid on;
title('Adaptive step-size history');

% ---- Simulink note ----
% In Simulink, build the same scalar ODE using:
%   Integrator block + Sum block + Sine Wave block + Gain(-2)
% and compare fixed-step "ode4 (Runge-Kutta)" vs variable-step "ode45".

%% Local functions
function yNext = rk2midpointStep(fun, t, y, h)
    k1 = fun(t, y);
    k2 = fun(t + 0.5*h, y + 0.5*h*k1);
    yNext = y + h*k2;
end

function yNext = rk4Step(fun, t, y, h)
    k1 = fun(t, y);
    k2 = fun(t + 0.5*h, y + 0.5*h*k1);
    k3 = fun(t + 0.5*h, y + 0.5*h*k2);
    k4 = fun(t + h, y + h*k3);
    yNext = y + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function [ts, ys] = integrateFixed(stepFun, fun, t0, tf, y0, h)
    ts = t0:h:tf;
    if ts(end) < tf
        ts = [ts tf];
    end
    ys = zeros(size(ts));
    ys(1) = y0;
    for i = 1:length(ts)-1
        hs = ts(i+1) - ts(i);
        ys(i+1) = stepFun(fun, ts(i), ys(i), hs);
    end
end

function res = integrateAdaptiveRK4StepDoubling(fun, t0, tf, y0, h0, atol, rtol, hMin, hMax, safety)
    t = t0; y = y0; h = h0;
    ts = t; ys = y; hHistory = [];
    accepted = 0; rejected = 0;

    while t < tf
        h = min(h, tf - t);
        if h < hMin
            error('Step size below hMin');
        end

        yFull = rk4Step(fun, t, y, h);
        yHalf = rk4Step(fun, t, y, 0.5*h);
        yHalf2 = rk4Step(fun, t + 0.5*h, yHalf, 0.5*h);

        errEst = abs(yHalf2 - yFull) / 15;
        scale = atol + rtol * max(abs(y), abs(yHalf2));
        errNorm = errEst / scale;

        if errNorm <= 1
            y = yHalf2 + (yHalf2 - yFull)/15;
            t = t + h;
            ts(end+1,1) = t; %#ok<AGROW>
            ys(end+1,1) = y; %#ok<AGROW>
            hHistory(end+1,1) = h; %#ok<AGROW>
            accepted = accepted + 1;

            if errNorm == 0
                factor = 2;
            else
                factor = safety * (1/errNorm)^(1/5);
            end
            factor = min(2, max(0.2, factor));
            h = min(hMax, factor*h);
        else
            rejected = rejected + 1;
            factor = safety * (1/max(errNorm,1e-16))^(1/5);
            factor = min(1, max(0.1, factor));
            h = max(hMin, factor*h);
        end
    end

    res.t = ts;
    res.y = ys;
    res.hHistory = hHistory;
    res.accepted = accepted;
    res.rejected = rejected;
end
