% Dangerous undetected failure rate (per hour)
lambda_DU = 1e-6;    % example value
T = 8760;            % proof test interval: 1 year in hours

% Approximate IEC 61508 low-demand formula
PFD_avg = lambda_DU * T / 2;

% Continuous mode PFH approximation
PFH = lambda_DU;

fprintf("PFD_avg = %.4g\n", PFD_avg);
fprintf("PFH     = %.4g failures per hour\n", PFH);

% In Simulink, these calculations can be implemented via Gain and Integrator
% blocks, and the resulting PFD_avg compared against SIL/PL target thresholds.
      
