
% Modular chain stiffness (series) and redundancy reliability
k = [3000, 2000, 1500];          % interface stiffnesses N/m
R = [0.98, 0.95, 0.97];          % module reliabilities

k_eq = 1 / sum(1 ./ k);
R_series = prod(R);

% Add parallel redundancy for module 2 (two identical backups)
R2_parallel = 1 - (1 - R(2))^2;
R_sys_redundant = R(1) * R2_parallel * R(3);

fprintf('Equivalent stiffness: %.2f N/m\\n', k_eq);
fprintf('Series reliability: %.5f\\n', R_series);
fprintf('Redundant reliability: %.5f\\n', R_sys_redundant);

% Simulink note:
% In Simscape Multibody, represent each module as a masked subsystem
% with standardized frame ports; reconfiguration becomes swapping subsystems.
      