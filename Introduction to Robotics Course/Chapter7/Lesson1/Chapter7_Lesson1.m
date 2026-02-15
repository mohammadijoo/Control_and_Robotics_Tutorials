Ts = 0.1;
A = [1 Ts; 0 1];
B = [0.5*Ts^2; Ts];

x = [0;1];
b = 0.05; sigma_p = 0.02; sigma_e = 0.05;
p_hat = 0;

N = 200;
p_true = zeros(N,1);
p_hat_hist = zeros(N,1);
p_extero = zeros(N,1);

for k=1:N
    u = 0;
    x = A*x + B*u;

    y_p = x(2) + b + sigma_p*randn; % proprio (velocity)
    y_e = x(1) + sigma_e*randn;     % extero (position)

    p_hat = p_hat + Ts*y_p;

    p_true(k) = x(1);
    p_hat_hist(k) = p_hat;
    p_extero(k) = y_e;
end

disp([p_true(end), p_hat_hist(end), p_extero(end)])
