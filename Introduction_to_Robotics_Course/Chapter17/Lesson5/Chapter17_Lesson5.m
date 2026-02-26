% Parameters
T = 0.1;      % sampling period [s]
v = 1.0;      % forward speed [m/s]

% State-space matrices for [e_y; e_psi]
A = [1, T*v;
     0, 1];
B = [0;
     T];

% Desired closed-loop poles (inside unit disc)
p1 = 0.8;
p2 = 0.6;
p_des = [p1, p2];

% Place poles to compute state feedback K
K = place(A, B, p_des);   % K is 1x2

% Closed-loop system
A_cl = A - B*K;

% Simulation of initial condition
x0 = [0.5; 0.2];  % initial lateral and heading errors
N  = 50;
x_hist = zeros(2, N+1);
x_hist(:,1) = x0;

for k = 1:N
    u = -K * x_hist(:,k);
    x_hist(:,k+1) = A * x_hist(:,k) + B * u;
end

t = 0:T:N*T;
figure;
subplot(2,1,1);
plot(t, x_hist(1,:)); grid on;
xlabel('Time [s]');
ylabel('e_y [m]');
title('Lateral error');

subplot(2,1,2);
plot(t, x_hist(2,:)); grid on;
xlabel('Time [s]');
ylabel('e_psi [rad]');
title('Heading error');
      
