function p = estimate_success_prob(numSamples, numTrials, dim, radius)
%ESTIMATE_SUCCESS_PROB Monte Carlo estimate of P(success)
%   numSamples : number of samples per trial
%   numTrials  : number of trials
%   dim        : configuration-space dimension
%   radius     : goal-tube radius

if nargin < 4
    radius = 0.1;
end
if nargin < 3
    dim = 4;
end
if nargin < 2
    numTrials = 1000;
end

center = 0.5 * ones(1, dim);
successCount = 0;

for t = 1:numTrials
    Q = rand(numSamples, dim);    % samples in [0,1]^dim
    diffs = Q - center;
    dists = sqrt(sum(diffs.^2, 2));
    if any(dists <= radius)
        successCount = successCount + 1;
    end
end

p = successCount / numTrials;
end

% Example usage:
for n = [10, 50, 100, 200, 500, 1000]
    p = estimate_success_prob(n, 1000, 4, 0.1);
    fprintf('n = %d, approx P(success) = %.3f\n', n, p);
end
      
