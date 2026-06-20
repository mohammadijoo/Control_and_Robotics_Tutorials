function [qExp, qDelta] = robustQualityMC(W0, sigmaW, numSamples, delta)
% W0: 3 x m matrix of nominal wrenches
% sigmaW: scalar wrench noise std
% numSamples: Monte Carlo samples
% delta: risk level (e.g., 0.1)

if nargin < 2, sigmaW = 0.05; end
if nargin < 3, numSamples = 200; end
if nargin < 4, delta = 0.1; end

[dim, m] = size(W0);
assert(dim == 3, 'Wrenches must be 3xM.');

qVals = zeros(numSamples, 1);

for s = 1:numSamples
    noise = sigmaW * randn(size(W0));
    W = W0 + noise;
    qVals(s) = ferrariCannyApprox(W, 64);
end

qExp = mean(qVals);
sortedQ = sort(qVals);
idx = max(1, min(length(sortedQ), floor(delta * length(sortedQ))));
qDelta = sortedQ(idx);

end

function qVal = ferrariCannyApprox(W, numDirs)
% Approximate Ferrari-Canny quality for planar wrenches.
if nargin < 2, numDirs = 64; end
[dim, m] = size(W);
qVal = inf;
for k = 1:numDirs
    d = randn(dim, 1);
    d = d / (norm(d) + 1e-12);
    supports = d' * W;   % 1 x m
    qDir = max(supports);
    if qDir < qVal
        qVal = qDir;
    end
end
end

% Example usage:
% W0 = [ 1.0  -1.0   0.8  -0.8;
%        0.8   0.8  -0.8  -0.8;
%        0.2  -0.2   0.3  -0.3 ];
% [qExp, qDelta] = robustQualityMC(W0, 0.05, 500, 0.1)
      
