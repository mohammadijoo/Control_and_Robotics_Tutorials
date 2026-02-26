function segments = dp_segment(phi, beta, maxLen)
% phi: T x d feature matrix
% beta: penalty per segment
% maxLen: optional maximum segment length

[T, ~] = size(phi);
if nargin < 3
    maxLen = T;
end

% Precompute segment costs
C = inf(T, T);
for i = 1:T
    maxJ = min(T, i + maxLen - 1);
    for j = i:maxJ
        seg = phi(i:j, :);
        mu = mean(seg, 1);
        centered = seg - mu;
        varSeg = sum(centered(:).^2) / numel(seg);
        C(i, j) = 0.5 * numel(seg) * log(varSeg);
    end
end

D = inf(T + 1, 1);
prev = -ones(T + 1, 1);
D(1) = 0.0;

for j = 2:T+1
    bestVal = inf;
    bestI = -1;
    for i = 1:j-1
        cost = D(i) + C(i, j - 1) + beta;
        if cost < bestVal
            bestVal = cost;
            bestI = i;
        end
    end
    D(j) = bestVal;
    prev(j) = bestI;
end

% Backtrack
segments = [];
j = T + 1;
while j > 1
    i = prev(j);
    segments = [i, j - 1; segments]; %#ok<AGROW>
    j = i;
end
end
      
