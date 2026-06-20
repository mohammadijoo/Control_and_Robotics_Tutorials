% Chapter21_Lesson4.m
% Minimum-phase vs non-minimum-phase systems in state-space.
% Requires Control System Toolbox for ss, tf, tzero, zero, step.

clear; clc; close all;

A = [0 1; -6 -5];
B = [0; 1];
D = 0;

% G_min(s) = (s + 1)/((s + 2)(s + 3))
Cmin = [1 1];
sysMin = ss(A, B, Cmin, D);

% G_nmp(s) = (-s + 1)/((s + 2)(s + 3)); zero at +1, DC gain positive.
Cnmp = [1 -1];
sysNmp = ss(A, B, Cnmp, D);

analyzeSystem('Minimum-phase example', sysMin);
analyzeSystem('Non-minimum-phase example', sysNmp);

figure;
step(sysMin, sysNmp, 8);
grid on;
legend('minimum phase', 'non-minimum phase', 'Location', 'best');
title('Step responses: RHP zero creates inverse initial motion');

function analyzeSystem(name, sys)
    fprintf('\n%s\n', name);
    z = tzero(sys);
    p = pole(sys);
    disp('Poles:'); disp(p.');
    disp('Transmission zeros:'); disp(z.');

    tol = 1e-8;
    if isempty(z)
        disp('Classification: minimum-phase, no finite zeros detected');
    elseif any(real(z) > tol)
        disp('Classification: non-minimum-phase, right-half-plane zero exists');
    elseif any(abs(real(z)) <= tol)
        disp('Classification: borderline non-minimum-phase, imaginary-axis zero exists');
    else
        disp('Classification: minimum-phase, all finite zeros are in the open left-half-plane');
    end

    [num, den] = tfdata(tf(sys), 'v');
    fprintf('Transfer numerator: '); disp(num);
    fprintf('Transfer denominator: '); disp(den);
end
