% Adjacency matrix for N = 8 states
N  = 8;
Adj = zeros(N, N);
Adj(1, [2 3]) = 1;  % 0 -> 1,2
Adj(2, 4)     = 1;  % 1 -> 3
Adj(3, [4 5]) = 1;  % 2 -> 3,4
Adj(4, 6)     = 1;  % 3 -> 5
Adj(5, [6 7]) = 1;  % 4 -> 5,6
Adj(6, 8)     = 1;  % 5 -> 7
Adj(7, 8)     = 1;  % 6 -> 7

S0  = [1];   % MATLAB indices for state 0
Bad = [7];   % state 6

% Forward reachability
reachable = false(N, 1);
queue     = S0(:);
reachable(S0) = true;

while ~isempty(queue)
    s = queue(1);
    queue(1) = [];
    succ = find(Adj(s, :) > 0);
    for k = 1:numel(succ)
        sn = succ(k);
        if ~reachable(sn)
            reachable(sn) = true;
            queue(end+1) = sn; %#ok<AGROW>
        end
    end
end

R_star = find(reachable) - 1;  % subtract 1 to get 0-based labels

% Backward reachability
backward = false(N, 1);
queue    = Bad(:);
backward(Bad) = true;

AdjT = Adj.';  % transpose for predecessors

while ~isempty(queue)
    s = queue(1);
    queue(1) = [];
    pred = find(AdjT(s, :) > 0);
    for k = 1:numel(pred)
        sp = pred(k);
        if ~backward(sp)
            backward(sp) = true;
            queue(end+1) = sp; %#ok<AGROW>
        end
    end
end

B_star = find(backward) - 1;

disp('Reachable from S0:');
disp(R_star);
disp('Backward reachable to Bad:');
disp(B_star);

safe = isempty(intersect(S0 - 1, B_star));
disp(['Safety holds? ', num2str(safe)]);

% In Simulink:
% - Use a Stateflow chart or a custom discrete-time model to generate the TS.
% - Export the adjacency matrix and integrate this script for offline verification.
      
