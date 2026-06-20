\
% Chapter20_Lesson5.m
% AMR Capstone final demo evaluation + research-style summary (MATLAB / Simulink)

clear; clc;
% [success, time_s, path_m, ref_m, rms_xy_m, clearance_m, collisions, latency_ms, energy_wh, map_iou]
B = [
  1 118.2 28.1 22.0 0.19 0.24 1 31.2 24.8 0.72;
  1 104.3 24.9 20.0 0.17 0.28 0 28.7 20.1 0.79;
  0 138.8 30.4 23.0 0.31 0.12 2 39.1 28.5 0.58;
  1 96.5  22.7 19.0 0.13 0.35 0 24.6 18.4 0.82;
  1 110.1 26.8 21.0 0.22 0.21 1 33.8 23.7 0.74
];
P = [
  1 91.4 24.0 22.0 0.11 0.31 0 22.3 21.2 0.84;
  1 86.0 21.8 20.0 0.10 0.34 0 20.4 18.8 0.87;
  1 104.2 25.6 23.0 0.16 0.22 1 24.9 22.1 0.76;
  1 80.9 20.5 19.0 0.09 0.39 0 18.9 17.3 0.90;
  1 89.1 23.5 21.0 0.14 0.27 0 21.8 19.6 0.83
];

AB = aggregateMetrics(B);
AP = aggregateMetrics(P);
disp(AB); disp(AP);

ciTime = pairedCI(P(:,2) - B(:,2));
ciIoU  = pairedCI(P(:,10) - B(:,10));

fprintf('Paired delta time: %.4f [%.4f, %.4f]\n', ciTime(1), ciTime(2), ciTime(3));
fprintf('Paired delta IoU : %.4f [%.4f, %.4f]\n', ciIoU(1), ciIoU(2), ciIoU(3));

% Simulink integration note:
% Export timeseries logs -> episode summary table -> run this script to compute final report metrics.

function A = aggregateMetrics(M)
    eff = M(:,4) ./ M(:,3);
    enpm = M(:,9) ./ M(:,3);
    A = struct();
    A.success_rate = mean(M(:,1));
    A.collision_free_rate = mean(M(:,7) == 0);
    A.time_s = mean(M(:,2));
    A.path_eff = mean(eff);
    A.rms_xy_m = mean(M(:,5));
    A.clearance_m = mean(M(:,6));
    A.latency_ms = mean(M(:,8));
    A.energy_wh_per_m = mean(enpm);
    A.map_iou = mean(M(:,10));
end

function ci = pairedCI(d)
    m = mean(d);
    s = std(d, 0);
    h = 1.959963984540054 * s / sqrt(numel(d));
    ci = [m, m - h, m + h];
end
