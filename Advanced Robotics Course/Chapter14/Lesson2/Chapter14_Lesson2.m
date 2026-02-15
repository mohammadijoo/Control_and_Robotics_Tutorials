N = 4;
L = N * eye(N) - ones(N);  % Laplacian for complete graph
alpha = 0.3;
P = eye(N) - alpha * L;

x = [0; 2; -1; 4];  % initial state
T = 30;
history = zeros(N, T + 1);
history(:, 1) = x;

for k = 1:T
    x = P * x;
    history(:, k + 1) = x;
end

disp('Final state:');
disp(x);
disp('Average of initial states:');
disp(mean(history(:, 1)));

% Plot trajectories
figure;
plot(0:T, history);
xlabel('k');
ylabel('x_i(k)');
legend('agent 1', 'agent 2', 'agent 3', 'agent 4');
grid on;
      
