
T_fast = 0.001;
m = 10;
T_slow = m * T_fast;
tf = 1.0;
N_steps = floor(tf / T_fast);

A_f = [1, T_fast;
       0, 1];
B_f = [0.5 * T_fast^2;
       T_fast];

K_fast = [0, 5];
K_slow = 50;

x = [0; 0];    % [q; qdot]
v_slow = 0;

q_hist = zeros(N_steps, 1);
t_hist = (0:N_steps-1)' * T_fast;

for k = 1:N_steps
    t = (k-1) * T_fast;

    if mod(k-1, m) == 0
        if t > 0.1
            q_des = 0.5;
        else
            q_des = 0.0;
        end
        e_q = q_des - x(1);
        v_slow = K_slow * e_q;
    end

    u = v_slow - K_fast * x;
    x = A_f * x + B_f * u;

    q_hist(k) = x(1);
end

plot(t_hist, q_hist);
xlabel('time [s]');
ylabel('joint position q');
title('Multi-rate joint control (fast inner, slow outer)');
grid on;
