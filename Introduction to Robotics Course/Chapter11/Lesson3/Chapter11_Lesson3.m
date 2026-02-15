% Discrete-time FIFO buffer simulation
B = 10;          % capacity
lambda = 50;     % producer Hz
mu = 40;         % consumer Hz
dt = 0.001;      % 1 ms step
T = 5;           % total time
q = 0;           % queue occupancy
drop = 0;

for t = 0:dt:T
    a = rand < lambda*dt; % Bernoulli arrival in dt
    b = rand < mu*dt;     % Bernoulli service in dt
    q_next = min(B, max(0, q + a - b));
    if q == B && a > b
        drop = drop + 1;
    end
    q = q_next;
end

fprintf("Final q=%d, drops=%d\n", q, drop);
      
