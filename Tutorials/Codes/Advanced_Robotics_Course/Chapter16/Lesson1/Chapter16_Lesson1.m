T = 0.1;
L = 1.0;
u_max = 0.5;
N_horizon = 10;
dx = 0.1;

controls = [-u_max, 0.0, u_max];
states = -L:dx:L;
initial_states = 0.0;  % could also use a vector of initial states

% Represent visited as logical matrix indexed by (state_index, step)
n_states = numel(states);
visited = false(n_states, N_horizon + 1);

% Map from x to index
[~, idx0] = min(abs(states - initial_states));
queue = [idx0, 0];  % [state_index, step]
visited(idx0, 1) = true;

safe = true;

while ~isempty(queue) && safe
    x_idx = queue(1,1);
    step  = queue(1,2);
    queue(1,:) = [];

    x = states(x_idx);
    if step == N_horizon
        continue;
    end

    for u = controls
        x_next = x + T * u;
        if x_next < -L || x_next > L
            fprintf('Unsafe successor from x=%f with u=%f\n', x, u);
            safe = false;
            break;
        end
        % snap to grid
        [~, idx_next] = min(abs(states - x_next));
        if ~visited(idx_next, step + 2)
            visited(idx_next, step + 2) = true;
            queue(end+1,:) = [idx_next, step + 1]; %#ok<AGROW>
        end
    end
end

fprintf('Abstraction declared SAFE up to horizon %d: %d\n', ...
        N_horizon, safe);
      
