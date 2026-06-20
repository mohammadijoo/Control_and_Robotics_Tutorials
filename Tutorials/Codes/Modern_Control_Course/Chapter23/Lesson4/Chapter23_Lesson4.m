% Chapter23_Lesson4.m
% Mapping time-domain specifications to desired poles for SISO pole placement.
% Includes from-scratch Ackermann implementation and MATLAB Control System Toolbox comparison.

clear; clc;

A = [0 1 0;
     0 0 1;
    -2 -3 -1];
b = [0; 0; 1];

Mp = 10;        % percent overshoot
Ts = 2.0;       % seconds, 2 percent settling time

[zeta, wn, pair] = desired_poles_from_specs(Mp, Ts);
sigma = -real(pair(1));
poles = [pair, -6*sigma];

K_ack = ackermann_gain(A, b, poles);
disp('zeta ='); disp(zeta);
disp('omega_n ='); disp(wn);
disp('desired poles ='); disp(poles);
disp('K from Ackermann ='); disp(K_ack);
disp('closed-loop eigenvalues ='); disp(eig(A - b*K_ack));

% If Control System Toolbox is installed, compare with acker/place:
if exist('acker', 'file') == 2
    K_toolbox = acker(A, b, poles);
    disp('K from acker ='); disp(K_toolbox);
end

function zeta = damping_ratio_from_overshoot(percent_overshoot)
    if percent_overshoot <= 0 || percent_overshoot >= 100
        error('percent_overshoot must be between 0 and 100');
    end
    m = percent_overshoot / 100;
    L = log(m);
    zeta = -L / sqrt(pi^2 + L^2);
end

function [zeta, wn, poles] = desired_poles_from_specs(percent_overshoot, settling_time)
    zeta = damping_ratio_from_overshoot(percent_overshoot);
    wn = 4 / (zeta * settling_time);
    sigma = zeta * wn;
    wd = wn * sqrt(max(0, 1 - zeta^2));
    poles = [-sigma + 1i*wd, -sigma - 1i*wd];
end

function Wc = controllability_matrix(A, b)
    n = size(A, 1);
    Wc = zeros(n, n);
    for k = 1:n
        Wc(:, k) = A^(k-1) * b;
    end
end

function K = ackermann_gain(A, b, poles)
    n = size(A, 1);
    Wc = controllability_matrix(A, b);
    if rank(Wc) < n
        error('(A,b) is not controllable');
    end
    coeff = poly(poles); % [1 a_{n-1} ... a0]
    phiA = A^n;
    for i = 1:n
        power = n - i;
        phiA = phiA + coeff(i+1) * A^power;
    end
    eT = zeros(1, n); eT(n) = 1;
    K = real(eT / Wc * phiA);
end
