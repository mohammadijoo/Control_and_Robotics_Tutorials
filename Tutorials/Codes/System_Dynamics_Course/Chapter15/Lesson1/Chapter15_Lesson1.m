% Chapter15_Lesson1.m
% Euler and Improved Euler (Heun) for y' = -2 y + sin(t), y(0)=1

clear; clc; close all;

f = @(t, y) -2*y + sin(t);
exact = @(t) (2*sin(t) - cos(t))/5 + (6/5)*exp(-2*t);

t0 = 0; tf = 5; y0 = 1;
hs = [0.2, 0.1, 0.05, 0.025];

eErr = zeros(size(hs));
hErr = zeros(size(hs));

fprintf('Step-size study (max error on [0,5])\n');
fprintf('%-10s %-16s %-16s\n', 'h', 'Euler', 'ImprovedEuler');

for i = 1:numel(hs)
    h = hs(i);
    [tE, yE] = eulerMethod(f, t0, tf, y0, h);
    [tH, yH] = improvedEulerMethod(f, t0, tf, y0, h);

    eErr(i) = max(abs(yE - exact(tE)));
    hErr(i) = max(abs(yH - exact(tH)));

    fprintf('%-10.3f %-16.8e %-16.8e\n', h, eErr(i), hErr(i));
end

fprintf('\nEstimated order (successive halving)\n');
for i = 1:numel(hs)-1
    pE = log(eErr(i)/eErr(i+1))/log(2);
    pH = log(hErr(i)/hErr(i+1))/log(2);
    fprintf('h=%.3f -> %.3f : Euler p~%.4f, Heun p~%.4f\n', hs(i), hs(i+1), pE, pH);
end

hPlot = 0.1;
[tE, yE] = eulerMethod(f, t0, tf, y0, hPlot);
[tH, yH] = improvedEulerMethod(f, t0, tf, y0, hPlot);
tDense = linspace(t0, tf, 800);

figure;
plot(tDense, exact(tDense), 'LineWidth', 1.5); hold on;
plot(tE, yE, 'o-');
plot(tH, yH, 's-');
xlabel('t'); ylabel('y(t)');
title('Euler vs Improved Euler');
legend('Exact', 'Euler (h=0.1)', 'Improved Euler (h=0.1)', 'Location', 'best');
grid on;

T = table(tH(:), yH(:), exact(tH(:))', abs(yH(:)-exact(tH(:))'), ...
    'VariableNames', {'t', 'Heun', 'Exact', 'AbsError'});
writetable(T, 'Chapter15_Lesson1_matlab_output.csv');

function [t, y] = eulerMethod(f, t0, tf, y0, h)
    n = round((tf - t0)/h);
    t = linspace(t0, tf, n+1);
    y = zeros(1, n+1);
    y(1) = y0;
    for k = 1:n
        y(k+1) = y(k) + h * f(t(k), y(k));
    end
end

function [t, y] = improvedEulerMethod(f, t0, tf, y0, h)
    n = round((tf - t0)/h);
    t = linspace(t0, tf, n+1);
    y = zeros(1, n+1);
    y(1) = y0;
    for k = 1:n
        s1 = f(t(k), y(k));
        yPred = y(k) + h*s1;
        s2 = f(t(k+1), yPred);
        y(k+1) = y(k) + 0.5*h*(s1 + s2);
    end
end
