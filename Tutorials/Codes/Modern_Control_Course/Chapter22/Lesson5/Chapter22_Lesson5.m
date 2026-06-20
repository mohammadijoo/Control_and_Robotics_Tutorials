% Chapter22_Lesson5.m
%
% Practical constraints and limitations of state feedback:
% simulation of ideal, saturated, rate-limited, and noisy state feedback.
%
% Optional Simulink note:
% The final section creates a minimal Simulink model if Simulink is installed.

clear; clc; close all;

A = [0 1; 0 0];
B = [0; 1];
K = [6 5];          % desired continuous poles -2 and -3 for double integrator
umax = 1.0;
maxRate = 8.0;
dt = 0.001;
tf = 6.0;
t = 0:dt:tf;
x0 = [1.2; 0.0];

[xIdeal, uIdeal] = simulateCase(A, B, K, umax, maxRate, dt, t, x0, false, false);
[xCon, uCon]     = simulateCase(A, B, K, umax, maxRate, dt, t, x0, true, false);
[xNoise, uNoise] = simulateCase(A, B, K, umax, maxRate, dt, t, x0, true, true);

Acl = A - B*K;
Q = eye(2);
P = lyap(Acl', Q);
rhoMax = umax^2 / (K * inv(P) * K');

disp('Lyapunov matrix P:');
disp(P);
disp('Guaranteed no-saturation ellipsoid radius rho <= ');
disp(rhoMax);

figure;
plot(t, xIdeal(1,:), 'DisplayName', 'ideal x1'); hold on;
plot(t, xCon(1,:), 'DisplayName', 'constrained x1');
plot(t, xNoise(1,:), 'DisplayName', 'constrained + noise x1');
grid on; xlabel('time [s]'); ylabel('position state');
legend('Location', 'best');

figure;
plot(t, uIdeal, 'DisplayName', 'ideal u'); hold on;
plot(t, uCon, 'DisplayName', 'saturated/rate-limited u');
plot(t, uNoise, 'DisplayName', 'saturated/rate-limited/noisy u');
grid on; xlabel('time [s]'); ylabel('control input');
legend('Location', 'best');

% Optional: create a minimal Simulink model with State-Space, Gain, Saturation,
% and Scope blocks. This section is skipped if Simulink is unavailable.
if exist('simulink', 'file') == 4
    modelName = 'Chapter22_Lesson5_Simulink';
    new_system(modelName);
    open_system(modelName);

    add_block('simulink/Continuous/State-Space', [modelName '/Plant']);
    set_param([modelName '/Plant'], 'A', mat2str(A), 'B', mat2str(B), ...
        'C', mat2str(eye(2)), 'D', mat2str([0;0]));

    add_block('simulink/Math Operations/Gain', [modelName '/StateFeedbackGain']);
    set_param([modelName '/StateFeedbackGain'], 'Gain', mat2str(-K), ...
        'Multiplication', 'Matrix(K*u)');

    add_block('simulink/Discontinuities/Saturation', [modelName '/Saturation']);
    set_param([modelName '/Saturation'], 'UpperLimit', num2str(umax), ...
        'LowerLimit', num2str(-umax));

    add_block('simulink/Sinks/Scope', [modelName '/Scope']);
    save_system(modelName);
    disp(['Created optional Simulink model: ' modelName '.slx']);
end

function [x, u] = simulateCase(A, B, K, umax, maxRate, dt, t, x0, constrained, noisy)
    x = zeros(2, numel(t));
    u = zeros(1, numel(t));
    x(:,1) = x0;
    rng(4);

    for k = 1:(numel(t)-1)
        xMeas = x(:,k);
        if noisy
            xMeas = xMeas + [0.02; 0.03] .* randn(2,1);
        end

        uCmd = -K * xMeas;
        if constrained
            uSat = min(max(uCmd, -umax), umax);
            if k == 1
                uPrev = 0;
            else
                uPrev = u(k-1);
            end
            step = maxRate * dt;
            u(k) = min(max(uSat, uPrev - step), uPrev + step);
        else
            u(k) = uCmd;
        end

        f = @(xx) A*xx + B*u(k);
        q1 = f(x(:,k));
        q2 = f(x(:,k) + 0.5*dt*q1);
        q3 = f(x(:,k) + 0.5*dt*q2);
        q4 = f(x(:,k) + dt*q3);
        x(:,k+1) = x(:,k) + (dt/6)*(q1 + 2*q2 + 2*q3 + q4);
    end
    u(end) = u(end-1);
end
