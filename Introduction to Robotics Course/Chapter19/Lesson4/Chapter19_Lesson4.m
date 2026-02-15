% Parameters
T = 0.05;                 % sampling period [s]
k_p = 0.8;                % proportional gain
d_safe = 0.4;             % safe distance [m]
N = 200;                  % number of steps
d = zeros(1, N+1);        % distance trajectory
u = zeros(1, N);          % control input

% Initial condition (safe)
d(1) = 1.2;

for k = 1:N
    if d(k) >= d_safe
        u(k) = k_p * (d(k) - d_safe);
    else
        u(k) = 0.0;
    end
    d(k+1) = d(k) - T * u(k);
end

% Plot results
k = 0:N;
figure; subplot(2,1,1);
plot(k*T, d);
hold on; yline(d_safe, '--');
xlabel('time [s]'); ylabel('distance d_k [m]');
legend('d_k', 'd_{safe}', 'Location', 'best');
grid on;

subplot(2,1,2);
stairs(k(1:end-1)*T, u);
xlabel('time [s]'); ylabel('velocity command u_k [m/s]');
grid on;
      
