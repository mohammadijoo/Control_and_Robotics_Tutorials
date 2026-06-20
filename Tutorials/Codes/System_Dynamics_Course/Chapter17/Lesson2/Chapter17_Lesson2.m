% Chapter17_Lesson2.m
% Autocorrelation, PSD, and White Noise Models (MATLAB / Simulink)

clear; clc; close all; rng(17);

N = 2048; sigma = 2.0; b = 0.6;
w = sigma * randn(N,1);             % white noise
x = filter([1 b], 1, w);            % x[n] = w[n] + b w[n-1]

M = 80;
RwFull = xcorr(w-mean(w), M, 'biased');
RxFull = xcorr(x-mean(x), M, 'biased');
Rw = RwFull(M+1:end);
Rx = RxFull(M+1:end);

Re = [Rx; Rx(end-1:-1:2)];
Sx = real(fft(Re));
omega = 2*pi*(0:length(Sx)-1)'/length(Sx);
SxTheory = sigma^2 * (1 + b^2 + 2*b*cos(omega));

fprintf('R_w(0) = %.6f\n', Rw(1));
fprintf('R_x(0) = %.6f\n', Rx(1));
fprintf('Theo R_x(0) = %.6f\n', sigma^2*(1+b^2));

figure; plot(0:M, Rw, 0:M, Rx); grid on;
xlabel('Lag m'); ylabel('Autocorrelation'); legend('White','Colored');

figure; half = 1:floor(length(Sx)/2);
plot(omega(half), Sx(half), omega(half), SxTheory(half), '--'); grid on;
xlabel('rad/sample'); ylabel('PSD'); legend('Estimated','Theory');

% Simulink setup:
% 1) Band-Limited White Noise block
% 2) Discrete Transfer Fcn with numerator [1 b], denominator [1]
% 3) Spectrum Analyzer + Scope
