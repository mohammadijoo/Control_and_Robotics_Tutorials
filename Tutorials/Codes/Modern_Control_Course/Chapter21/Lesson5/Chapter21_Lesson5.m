% Chapter21_Lesson5.m
% Impact of zeros on achievable performance and limitations.
% Requires MATLAB Control System Toolbox for tf, zero, step, evalfr.

clear; clc; close all;

s = tf('s');
Gmin = 6*(s + 1)/(s^2 + 5*s + 6);   % zero at -1, DC gain 1
Gnmp = 6*(1 - s)/(s^2 + 5*s + 6);   % zero at +1, DC gain 1

fprintf('Minimum-phase zero:\n');
disp(zero(Gmin));
fprintf('Non-minimum-phase zero:\n');
disp(zero(Gnmp));
fprintf('Common poles:\n');
disp(pole(Gnmp));

figure;
step(Gmin, Gnmp, 8);
grid on;
legend('zero at -1: minimum phase', 'zero at +1: non-minimum phase', 'Location', 'best');
title('RHP zero produces inverse response and limits fast tracking');

% Interpolation constraint at the RHP zero z=1.
% For unity feedback L(s)=K*G(s), T=L/(1+L), S=1/(1+L).
% Since Gnmp(1)=0, every internally stable design without unstable cancellation
% satisfies T(1)=0 and S(1)=1.
z = 1;
for K = [0.5 2 10 100]
    L = K*Gnmp;
    T = feedback(L, 1);
    S = feedback(1, L);
    fprintf('K = %7.2f,  S(z) = %.8f,  T(z) = %.8f\n', ...
        K, real(evalfr(S, z)), real(evalfr(T, z)));
end

% Optional: compare closed-loop step responses under proportional gain.
figure;
hold on;
for K = [0.5 2 10]
    Tnmp = feedback(K*Gnmp, 1);
    step(Tnmp, 8);
end
grid on;
legend('K=0.5', 'K=2', 'K=10', 'Location', 'best');
title('Increasing gain cannot remove the RHP-zero interpolation constraint');
hold off;
