rng(0);
A = [0 1; -2 -0.5];
B = [0; 1];

T  = 0.01;      % nominal sample time
Jm = 0.002;     % max jitter
N  = 500;

delta = (2*rand(N,1)-1)*Jm;
t_k = (0:N-1)'*T + delta;
T_k = diff(t_k);

Phi_nom = expm(A*T);

Phi_var = zeros(2,2,N-1);
for k=1:N-1
    Phi_var(:,:,k) = expm(A*T_k(k));
end

disp("Nominal Phi:");
disp(Phi_nom);
disp("Mean Phi under jitter:");
disp(mean(Phi_var,3));
disp("Std of Phi entries:");
disp(std(reshape(Phi_var,4,[]),0,2));
