% Chapter20_Lesson4.m
% Navigation Stack Deployment (Capstone AMR)
% MATLAB/Simulink-oriented deployment analysis: timing budget, inflation, and tracking stability.
clear; clc;

%% 1) Timing budget (single-core utilization check)
period_ms = 100;
tasks = struct( ...
    'sensor_preprocess', 8, ...
    'localization', 14, ...
    'costmap_update', 18, ...
    'global_planner', 12, ...
    'local_planner', 22, ...
    'controller', 5, ...
    'bt_tick', 3);

wcet_values = struct2array(tasks);
U = sum(wcet_values / period_ms);
fprintf('Period = %.1f ms, Utilization = %.3f, schedulable = %d\n', period_ms, U, U <= 1.0);

%% 2) Costmap inflation profile
r_ins = 0.22;
r_inf = 0.70;
c_lethal = 254;
c_ins = 220;
kappa = 8.0;

d = linspace(0, 1.0, 200);
C = zeros(size(d));
for i = 1:numel(d)
    if d(i) <= r_ins
        C(i) = c_lethal;
    elseif d(i) >= r_inf
        C(i) = 0;
    else
        C(i) = c_ins * exp(-kappa * (d(i) - r_ins));
    end
end

figure('Name', 'Chapter20_Lesson4 Inflation Profile');
plot(d, C, 'LineWidth', 2); grid on;
xlabel('Distance to obstacle center (m)');
ylabel('Inflation cost');
title('Inflation Decay for Local Costmap');

%% 3) Simple path-tracking error dynamics (small-angle approximation)
% e_y_dot = v * e_psi
% e_psi_dot = -k_y * e_y - k_psi * e_psi
v = 0.6; 
k_y = 1.8; 
k_psi = 2.4;

A = [0, v; -k_y, -k_psi];
eigA = eig(A);
disp('Closed-loop eigenvalues for lateral-heading subsystem:');
disp(eigA);

% Simulate error convergence
dt = 0.01;
T = 8;
t = 0:dt:T;
x = zeros(2, numel(t));
x(:,1) = [0.35; 0.25]; % [e_y; e_psi]

for k = 1:numel(t)-1
    xdot = A * x(:,k);
    x(:,k+1) = x(:,k) + dt * xdot;
end

V = 0.5 * (k_y * x(1,:).^2 + v * x(2,:).^2);

figure('Name', 'Chapter20_Lesson4 Tracking Error');
plot(t, x(1,:), 'LineWidth', 2); hold on;
plot(t, x(2,:), 'LineWidth', 2); grid on;
xlabel('Time (s)');
ylabel('Error');
legend('e_y', 'e_\psi');
title('Lateral and Heading Error Convergence');

figure('Name', 'Chapter20_Lesson4 Lyapunov');
plot(t, V, 'LineWidth', 2); grid on;
xlabel('Time (s)');
ylabel('V(e)');
title('Lyapunov Function Decrease');

%% 4) Recovery trigger logic (suitable for Stateflow/Simulink logic implementation)
planner_runtimes_ms = [60 72 91 88 95 65 77 83 90];
max_consecutive_miss = 3;
deadline_ms = 80;
miss_count = 0;

for k = 1:numel(planner_runtimes_ms)
    if planner_runtimes_ms(k) > deadline_ms
        miss_count = miss_count + 1;
    else
        miss_count = 0;
    end

    if miss_count >= max_consecutive_miss
        fprintf('Step %d: Recovery -> clear local costmap + stop + retry\n', k);
        miss_count = 0;
    else
        fprintf('Step %d: OK (runtime %.1f ms, miss_count=%d)\n', k, planner_runtimes_ms(k), miss_count);
    end
end

%% 5) Simulink deployment note
% Recommended block partition:
% [Sensor Fusion] -> [Localization] -> [Global Planner] -> [Local Planner] -> [Controller]
% Add a Stateflow chart for lifecycle and recovery states:
%   INIT -> CONFIGURE -> ACTIVATE -> RUN -> RECOVERY -> RUN -> SHUTDOWN
