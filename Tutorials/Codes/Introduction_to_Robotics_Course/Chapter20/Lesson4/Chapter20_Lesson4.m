% Time vector and signals (for example, exported from Simulink)
T = 10.0;
N = 1001;
t = linspace(0.0, T, N);
r = ones(size(t));                         % step reference
y = 1.0 - exp(-t) .* cos(2.0 * t);        % example response
e = r - y;

dt = t(2) - t(1);

IAE  = sum(abs(e))      * dt;
ISE  = sum(e.^2)        * dt;
ITAE = sum(t .* abs(e)) * dt;
RMSE = sqrt(mean(e.^2));
maxErr = max(abs(e));

fprintf('IAE  = %.4f\n', IAE);
fprintf('ISE  = %.4f\n', ISE);
fprintf('ITAE = %.4f\n', ITAE);
fprintf('RMSE = %.4f\n', RMSE);
fprintf('max |e| = %.4f\n', maxErr);

% For multiple runs, store metrics in a vector and compute statistics
numRuns = 10;
rmseRuns = zeros(numRuns,1);
for i = 1:numRuns
    noise = 0.01 * randn(size(e));
    ei = e + noise;
    rmseRuns(i) = sqrt(mean(ei.^2));
end

rmseMean = mean(rmseRuns);
rmseStd  = std(rmseRuns, 1);  % population std

fprintf('Mean RMSE over runs = %.4f\n', rmseMean);
fprintf('Std of RMSE         = %.4f\n', rmseStd);
      
