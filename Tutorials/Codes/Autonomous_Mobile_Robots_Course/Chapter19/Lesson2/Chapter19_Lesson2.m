% Chapter19_Lesson2.m
% Metrics for Navigation Robustness (Autonomous Mobile Robots)
%
% This MATLAB script computes robustness metrics from a navigation log table.
% It also includes a Simulink-compatible helper function for online metric updates.
%
% Required columns:
% episode_id, time_s, x, y, goal_x, goal_y, clearance_m, collision, intervention,
% goal_reached, tracking_error_m, cmd_v, cmd_v_max, cmd_w, cmd_w_max, recovery_event
%
% Robotics toolbox notes:
% - Robotics System Toolbox / ROS Toolbox can be used to import rosbag logs.
% - In Simulink, connect signals to a MATLAB Function block that calls
%   Chapter19_Lesson2_stepMetric() for cycle-level scoring.

clear; clc;

if exist('Chapter19_Lesson2_log.csv', 'file')
    T = readtable('Chapter19_Lesson2_log.csv');
else
    T = localGenerateSyntheticLog(12);
    writetable(T, 'Chapter19_Lesson2_log.csv');
    fprintf('Wrote synthetic log to Chapter19_Lesson2_log.csv\n');
end

episodes = unique(T.episode_id);
N = numel(episodes);

M = table('Size',[N 13], ...
    'VariableTypes', {'double','double','double','double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'episode_id','success','collision_free','intervention_free','had_failure','recovered_after_failure', ...
                      'completion_time_s','path_length_m','reference_distance_m','path_efficiency','min_clearance_m', ...
                      'tracking_rmse_m','saturation_ratio'});

for ii = 1:N
    ep = episodes(ii);
    G = sortrows(T(T.episode_id == ep, :), 'time_s');

    success = any(G.goal_reached > 0);
    collision_free = all(G.collision == 0);
    intervention_free = all(G.intervention == 0);
    had_failure = any((G.collision + G.intervention) > 0);
    recovered_after_failure = had_failure && any(G.recovery_event > 0) && success;

    dt = G.time_s(end) - G.time_s(1);
    dx = diff(G.x); dy = diff(G.y);
    L = sum(sqrt(dx.^2 + dy.^2));
    dref = hypot(G.goal_x(1)-G.x(1), G.goal_y(1)-G.y(1));
    eta = 0;
    if success
        eta = min(1, dref / max(L, eps));
    end

    min_clear = min(G.clearance_m);
    track_rmse = sqrt(mean(G.tracking_error_m.^2));
    sat = mean(abs(G.cmd_v) >= 0.98*max(G.cmd_v_max, eps) | abs(G.cmd_w) >= 0.98*max(G.cmd_w_max, eps));

    M{ii,:} = [ep, success, collision_free, intervention_free, had_failure, recovered_after_failure, ...
               dt, L, dref, eta, min_clear, track_rmse, sat];
end

k_success = sum(M.success);
k_collision_free = sum(M.collision_free);
k_intervention_free = sum(M.intervention_free);
n_failed = sum(M.had_failure);
k_recovered = sum(M.recovered_after_failure);

[p_succ, lo_succ, hi_succ] = localWilson(k_success, N, 1.96);
[p_safe, lo_safe, hi_safe] = localWilson(k_collision_free, N, 1.96);

time_score = mean(exp(-M.completion_time_s/60) .* M.success);
track_score = mean(exp(-M.tracking_rmse_m/0.25));
clearance_score = mean(min(max(M.min_clearance_m/0.30, 0), 1));
eff_score = mean(M.path_efficiency);

robustness = 0.30*p_safe + 0.15*(k_intervention_free/N) + 0.20*p_succ + ...
             0.15*clearance_score + 0.10*eff_score + 0.05*track_score + 0.05*time_score;

fprintf('\nPer-episode metrics:\n');
disp(M);

fprintf('\nSummary metrics:\n');
fprintf('Success rate               = %.4f (Wilson95%% [%.4f, %.4f])\n', p_succ, lo_succ, hi_succ);
fprintf('Collision-free rate        = %.4f (Wilson95%% [%.4f, %.4f])\n', p_safe, lo_safe, hi_safe);
fprintf('Intervention-free rate     = %.4f\n', k_intervention_free/N);
fprintf('Mean completion time (succ)= %.4f s\n', mean(M.completion_time_s(M.success>0)));
fprintf('Mean path efficiency       = %.4f\n', mean(M.path_efficiency));
fprintf('Mean min clearance         = %.4f m\n', mean(M.min_clearance_m));
fprintf('Mean tracking RMSE         = %.4f m\n', mean(M.tracking_rmse_m));
if n_failed > 0
    fprintf('Recovery-after-failure rate= %.4f\n', k_recovered/n_failed);
else
    fprintf('Recovery-after-failure rate= NaN\n');
end
fprintf('Composite robustness score = %.4f\n', robustness);

writetable(M, 'Chapter19_Lesson2_episode_metrics_matlab.csv');

%% Simulink-friendly helper (can be used inside a MATLAB Function block)
% score_t = Chapter19_Lesson2_stepMetric(clearance_m, track_err_m, collision, intervention)

function score_t = Chapter19_Lesson2_stepMetric(clearance_m, track_err_m, collision, intervention)
%#codegen
clear_score = min(max(clearance_m / 0.30, 0), 1);
track_score = exp(-track_err_m / 0.25);
safety_gate = 1.0;
if (collision ~= 0) || (intervention ~= 0)
    safety_gate = 0.0;
end
score_t = safety_gate * (0.7 * clear_score + 0.3 * track_score);
end

function T = localGenerateSyntheticLog(numEpisodes)
rows = [];
for ep = 0:(numEpisodes-1)
    x = 0; y = 0;
    gx = 9 + 0.2*ep; gy = mod(ep,3)-1;
    for k = 0:180
        t = 0.1*k;
        d = hypot(gx-x, gy-y);
        th = atan2(gy-y, gx-x);
        vMax = 0.8; wMax = 1.2;
        v = min(vMax, 0.55 + 0.05*sin(0.07*k + ep));
        w = max(-wMax, min(wMax, 0.32*sin(0.05*k)));
        x = x + 0.1*v*cos(th);
        y = y + 0.1*v*sin(th);

        clearance = 0.7 + 0.2*sin(0.1*k + ep);
        if ep == 4 && k > 80 && k < 95
            clearance = clearance - 0.60;
        end
        if ep == 9 && k > 55 && k < 70
            clearance = clearance - 0.45;
        end
        clearance = max(0, clearance);

        collision = double(clearance < 0.05);
        intervention = double(clearance < 0.12 && collision == 0);
        goal_reached = double(d < 0.33 && collision == 0);
        tracking_error_m = abs(0.05*sin(0.02*k + 0.1*ep));
        recovery_event = double(intervention == 1 && mod(k,6) == 0);

        rows = [rows; ep, t, x, y, gx, gy, clearance, collision, intervention, goal_reached, ...
                     tracking_error_m, v, vMax, w, wMax, recovery_event]; %#ok<AGROW>
        if collision || goal_reached
            break;
        end
    end
end

T = array2table(rows, 'VariableNames', ...
    {'episode_id','time_s','x','y','goal_x','goal_y','clearance_m','collision','intervention', ...
     'goal_reached','tracking_error_m','cmd_v','cmd_v_max','cmd_w','cmd_w_max','recovery_event'});
end

function [p, lo, hi] = localWilson(k, n, z)
if n <= 0
    p = NaN; lo = NaN; hi = NaN; return;
end
p = k / n;
den = 1 + z^2/n;
ctr = (p + z^2/(2*n)) / den;
rad = (z/den) * sqrt(p*(1-p)/n + z^2/(4*n^2));
lo = ctr - rad;
hi = ctr + rad;
end
