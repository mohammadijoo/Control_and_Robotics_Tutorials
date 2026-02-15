% Load logs exported from the experiment (e.g., from Simulink Data Inspector)
data = readtable("robot_logs.csv");

% Error signal
data.e = data.y_ref - data.y_meas;

trial_ids = unique(data.trial_id);
J_values = zeros(numel(trial_ids), 1);

for i = 1:numel(trial_ids)
    tid = trial_ids(i);
    idx = data.trial_id == tid;
    e = data.e(idx);
    J_values(i) = sqrt(mean(e.^2));
    fprintf("Trial %d RMS error = %.4f\n", tid, J_values(i));
end

J_mean = mean(J_values);
J_std  = std(J_values, 1);   % population std; for sample use std(J_values, 0)

fprintf("Mean RMS error = %.4f\n", J_mean);
fprintf("Std of RMS error = %.4f\n", J_std);

% Example: if J_values come from a Simulink model run multiple times
% for different seeds, this script can be placed in a Live Script that
% also embeds plots and report text.
      
