T = 0.01;
T_end = 10.0;
k_gain = 1.0;
alpha = 0.4;
U_max = 2.0;

N = floor(T_end / T);
t = linspace(0, T_end, N + 1);
x = zeros(1, N + 1);

human_command = @(time) (time < 5.0) * 1.0 + (time >= 5.0) * (-1.0);

for k = 1:N
    u_h = human_command(t(k));
    u_a = -k_gain * x(k);
    u = alpha * u_h + (1 - alpha) * u_a;
    % Saturation
    u = max(-U_max, min(U_max, u));
    x(k + 1) = x(k) + T * u;
end

plot(t, x);
xlabel('Time [s]');
ylabel('Position x(t)');
title('1-DOF Shared Autonomy Simulation');
grid on;
      
