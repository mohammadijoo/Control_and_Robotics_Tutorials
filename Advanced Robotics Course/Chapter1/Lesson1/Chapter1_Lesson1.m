function path = cspace_interpolate(q_start, q_goal, periodic, num_samples)
%CSpace interpolation with optional periodic coordinates.
% q_start, q_goal: 1-by-n
% periodic: 1-by-n logical indicating periodic coordinates
% num_samples: scalar number of samples along the path

q_start = wrap_angles(q_start, periodic);
diff = shortest_difference(q_start, q_goal, periodic);
alphas = linspace(0.0, 1.0, num_samples);
n = numel(q_start);
path = zeros(num_samples, n);
for k = 1:num_samples
    a = alphas(k);
    q = q_start + a * diff;
    q = wrap_angles(q, periodic);
    path(k, :) = q;
end
end

function q = wrap_angles(q, periodic)
% Wrap periodic coordinates to (-pi, pi]
for i = 1:numel(q)
    if periodic(i)
        x = q(i);
        x = mod(x + pi, 2.0 * pi);
        if x < 0.0
            x = x + 2.0 * pi;
        end
        q(i) = x - pi;
    end
end
end

function diff = shortest_difference(q_from, q_to, periodic)
q_from = wrap_angles(q_from, periodic);
q_to   = wrap_angles(q_to, periodic);
diff   = q_to - q_from;
for i = 1:numel(diff)
    if periodic(i)
        if diff(i) > pi
            diff(i) = diff(i) - 2.0 * pi;
        elseif diff(i) < -pi
            diff(i) = diff(i) + 2.0 * pi;
        end
    end
end
end

% Example usage (2R arm):
% periodic = [true, true];
% q_start = [0, 0];
% q_goal  = [pi, -pi];
% path = cspace_interpolate(q_start, q_goal, periodic, 5);

% Simulink remark:
% A Simulink implementation would represent q_start, q_goal, and periodic as inputs,
% implement wrap_angles and shortest_difference as MATLAB Function blocks, and
% generate a sequence of configurations by iterating alpha in a For Iterator Subsystem.
      
