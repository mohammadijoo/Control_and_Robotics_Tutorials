function [V, policy] = robust_value_iteration(Pmin, Pmax, cost, gamma, tol, maxIter)
% Pmin, Pmax: S x A x S arrays (interval transitions)
% cost: S x A matrix
% gamma: discount factor

if nargin < 5, tol = 1e-6; end
if nargin < 6, maxIter = 200; end

[S, A, ~] = size(Pmin);
V = zeros(S, 1);
Vnew = zeros(S, 1);

for it = 1:maxIter
    diff = 0;
    for s = 1:S
        best = inf;
        for a = 1:A
            % inner worst-case expectation
            v = V;
            [~, order] = sort(-v);  % descending
            p = zeros(S, 1);
            remaining = 1.0;
            for idx = 1:S
                sp = order(idx);
                maxAllow = min(Pmax(s, a, sp), remaining);
                minAllow = Pmin(s, a, sp);
                alloc = max(minAllow, maxAllow);
                alloc = min(alloc, remaining);
                p(sp) = alloc;
                remaining = remaining - alloc;
                if remaining <= 1e-12
                    break;
                end
            end
            if remaining > 1e-12
                free = find(p < squeeze(Pmax(s, a, :)) - 1e-12);
                if ~isempty(free)
                    extra = remaining / numel(free);
                    for k = 1:numel(free)
                        sp = free(k);
                        p(sp) = min(Pmax(s, a, sp), p(sp) + extra);
                    end
                end
            end
            worst = p' * V;
            q = cost(s, a) + gamma * worst;
            if q < best
                best = q;
            end
        end
        Vnew(s) = best;
        diff = max(diff, abs(Vnew(s) - V(s)));
    end
    V = Vnew;
    if diff < tol
        break;
    end
end

policy = zeros(S, 1);
for s = 1:S
    best = inf;
    bestA = 1;
    for a = 1:A
        v = V;
        [~, order] = sort(-v);
        p = zeros(S, 1);
        remaining = 1.0;
        for idx = 1:S
            sp = order(idx);
            maxAllow = min(Pmax(s, a, sp), remaining);
            minAllow = Pmin(s, a, sp);
            alloc = max(minAllow, maxAllow);
            alloc = min(alloc, remaining);
            p(sp) = alloc;
            remaining = remaining - alloc;
            if remaining <= 1e-12
                break;
            end
        end
        if remaining > 1e-12
            free = find(p < squeeze(Pmax(s, a, :)) - 1e-12);
            if ~isempty(free)
                extra = remaining / numel(free);
                for k = 1:numel(free)
                    sp = free(k);
                    p(sp) = min(Pmax(s, a, sp), p(sp) + extra);
                end
            end
        end
        worst = p' * V;
        q = cost(s, a) + gamma * worst;
        if q < best
            best = q;
            bestA = a;
        end
    end
    policy(s) = bestA;
end
end
      
