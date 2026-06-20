function [V, pi] = soft_value_iteration(theta, P, gamma, nIter)
% theta: k-by-1 reward parameters
% P: nS-by-nA-by-nS transition probabilities
% Here we assume state features are one-hot, so R(s) = theta(s).

nS = size(P, 1);
nA = size(P, 2);

R = theta(:);          % nS-by-1
V = zeros(nS, 1);

for it = 1:nIter
    Q = zeros(nS, nA);
    for s = 1:nS
        for a = 1:nA
            Q(s,a) = R(s) + gamma * squeeze(P(s,a,:))' * V;
        end
    end
    V = log(sum(exp(Q), 2) + 1e-8);
end

Q = zeros(nS, nA);
for s = 1:nS
    for a = 1:nA
        Q(s,a) = R(s) + gamma * squeeze(P(s,a,:))' * V;
    end
end

pi = exp(Q - V);
pi = pi ./ sum(pi, 2);
end
      
