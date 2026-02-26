% Example: log signals and compute residual statistic in MATLAB

Ts = 0.01;
A = [1 Ts; 0 1];
B = [0; Ts];
C = [1 0];
L = [0.2; 2.0];

S = 0.04;          % residual variance
gamma = 6.635;     % chi-square(1,0.99)

xhat = [0;0];

N = 100;
log_u = zeros(N,1);
log_y = zeros(N,1);
log_J = zeros(N,1);

for k=1:N
    u = 0.1;
    y = 0.0;

    r = y - C*xhat;
    xhat = A*xhat + B*u + L*r;

    J = (r'*r)/S;

    log_u(k)=u; log_y(k)=y; log_J(k)=J;

    if J > gamma
        fprintf("Fault detected at k=%d, J=%.3f\n", k, J);
    end
end

% Visualize logs
figure; plot((0:N-1)*Ts, log_J); grid on;
xlabel("time (s)"); ylabel("J_k");
title("Residual statistic over time");
      
