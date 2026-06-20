% States are 1..n; adjacency matrix A(i,j)=1 if transition i->j allowed
A = [0 1 1 0;
     0 0 1 1;
     0 0 1 0;
     0 0 0 1];
labels = { {}, {}, {'goal'}, {'collision'} };
init = 1;
maxDepth = 5;

function ok = check_trace(seq, labels)
    hasGoal = false;
    for k = 1:numel(seq)
        Lk = labels{seq(k)};
        if ismember('collision', Lk)
            ok = false; return;
        end
        if ismember('goal', Lk)
            hasGoal = true;
        end
    end
    ok = hasGoal;
end

function ok = dfs_all_paths(A, labels, state, depth, seq, maxDepth)
    seq(end+1) = state; %#ok<AGROW>
    if depth == maxDepth
        ok = check_trace(seq, labels);
        return;
    end
    % If no outgoing edges, treat as stutter (stay in same state)
    succ = find(A(state,:) ~= 0);
    if isempty(succ)
        succ = state;
    end
    ok = true;
    for s = succ
        if ~dfs_all_paths(A, labels, s, depth+1, seq, maxDepth)
            ok = false; return;
        end
    end
end

all_ok = dfs_all_paths(A, labels, init, 0, [], maxDepth);
disp(all_ok);
      
