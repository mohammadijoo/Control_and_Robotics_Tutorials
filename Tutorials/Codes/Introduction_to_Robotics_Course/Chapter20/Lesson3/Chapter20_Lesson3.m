% Number of subsystems (e.g., mechanics, control, perception)
n = 3;
Nsteps = 10;

% Initial progress x_0
x = zeros(n, 1);

% Effort-to-progress matrix B (n x n) and constant weekly effort u_k
B = [0.2  0.0  0.0;
     0.1  0.15 0.0;
     0.0  0.1  0.2];

u = [1.0; 1.0; 1.0];  % normalized effort per week

% Milestone region C x_k >= d
C = eye(n);
d = 0.8 * ones(n, 1);

km = NaN;  % first hitting time
for k = 0:(Nsteps - 1)
    % Check milestone at current state
    if all(C * x >= d) && isnan(km)
        km = k;
    end

    % Update progress
    x = x + B * u;
    x = min(x, 1.0);  % clip at 1.0 (fully complete)
end

if isnan(km)
    fprintf("Milestone not reached in %d steps.\n", Nsteps);
else
    fprintf("Milestone reached at step k = %d.\n", km);
end
      
