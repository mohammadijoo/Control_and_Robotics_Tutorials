
% Continuous-time closed-loop system: x_dot = A_cl * x
J = 0.05;
b = 0.01;
k_p = 50;
k_d = 2;

A_cl = [ 0,       1;
        -k_p/J, -(k_d + b)/J ];

Ts = 1e-3;

% Discretization with zero-order hold
sys_c = ss(A_cl, [], eye(2), []);
sys_d = c2d(sys_c, Ts, 'zoh');

A_d = sys_d.A;
eigAd = eig(A_d);

disp('A_d = ');
disp(A_d);
disp('Eigenvalues(A_d) = ');
disp(eigAd);

% Simulated response for an initial error
x0 = [0.1; 0.0];
Tfinal = 5.0;
N = round(Tfinal / Ts);
t_vec = (0:N-1)' * Ts;

% Since no explicit input in error coordinates, we use zero input
u_seq = zeros(N, 1);
[y_sim, ~, x_sim] = lsim(sys_d, u_seq, t_vec, x0);

% Energy-like quantity V_k = x_k' P x_k with P = I
V = sum(x_sim.^2, 2);

figure;
subplot(3,1,1); plot(t_vec, x_sim(:,1)); ylabel('e_q');
subplot(3,1,2); plot(t_vec, x_sim(:,2)); ylabel('e_dq');
subplot(3,1,3); plot(t_vec, V); ylabel('V'); xlabel('t (s)');
