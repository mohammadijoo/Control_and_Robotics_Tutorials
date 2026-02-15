
% Run the Simulink model
mdl = 'capstone_demo';
load_system(mdl);

% Set simulation parameters if needed
set_param(mdl, 'StopTime', 'Tf');  % assume Tf defined in workspace

simOut = sim(mdl, 'SrcWorkspace', 'current');

% Assume simOut contains logs: q, qd, tau (timeseries)
q_ts  = simOut.q;
qd_ts = simOut.qd;
tau_ts = simOut.tau;

t = q_ts.Time;
q = q_ts.Data;   % N-by-n
qd = qd_ts.Data; % N-by-n
tau = tau_ts.Data;

Ts = mean(diff(t));
e = qd - q;

% ISE approximation
J_ISE = Ts * sum(vecnorm(e(1:end-1, :), 2, 2).^2);

% RMS per joint
Ttotal = Ts * (size(e, 1) - 1);
sq_int = Ts * sum(e(1:end-1, :).^2, 1);
rms_joints = sqrt(sq_int / Ttotal);

% Quadratic effort with identity weighting
J_u = Ts * sum(vecnorm(tau(1:end-1, :), 2, 2).^2);

fprintf('ISE = %.4f\n', J_ISE);
fprintf('RMS per joint:\n');
disp(rms_joints);
fprintf('Effort J_u = %.4f\n', J_u);
