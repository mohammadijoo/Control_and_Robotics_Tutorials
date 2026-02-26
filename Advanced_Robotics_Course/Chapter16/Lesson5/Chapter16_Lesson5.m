function [W, strategy] = cbcPlanner(safe, goal, delta)
% safe, goal: logical row vectors of length nStates
% delta: cell array of size nStates
%   delta{s} is a struct array with fields:
%     .actionId : integer
%     .succ     : row vector of successor state indices

nStates = numel(safe);
W = false(1, nStates);

% W^0 = Safe ∩ Goal
W(safe & goal) = true;

changed = true;
while changed
    changed = false;
    for s = 1:nStates
        if ~safe(s) || W(s)
            continue;
        end
        trList = delta{s};
        canControl = false;
        for k = 1:numel(trList)
            succ = trList(k).succ;
            if isempty(succ), continue; end
            if all(W(succ))
                canControl = true;
                break;
            end
        end
        if canControl
            W(s) = true;
            changed = true;
        end
    end
end

strategy = -1 * ones(1, nStates);
for s = 1:nStates
    if ~W(s), continue; end
    trList = delta{s};
    for k = 1:numel(trList)
        succ = trList(k).succ;
        if isempty(succ), continue; end
        if all(W(succ))
            strategy(s) = trList(k).actionId;
            break;
        end
    end
end
end
      
