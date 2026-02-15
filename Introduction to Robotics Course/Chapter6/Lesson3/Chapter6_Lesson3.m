% Parameters
A = 3.0e-3;
beta_e = 1.2e9;
Vt = 6.0e-5;
m = 8.0;
b = 120.0;
Kq = 2.5e-5;
Kp = 1.0e-11;
Ct = 5.0e-12;

% State-space matrices from Section 5
Axs = [0 1 0;
       0 -b/m A/m;
       0 -(A*beta_e)/Vt -((Kp+Ct)*beta_e)/Vt];
Bxs = [0; 0; (Kq*beta_e)/Vt];

sys = ss(Axs, Bxs, eye(3), zeros(3,1));
t = 0:1e-3:1;
u = 2*(t>=0.1);  % step at 0.1 s

[y, t_out, x_out] = lsim(sys, u, t);

figure; plot(t_out, x_out(:,1));
xlabel('time [s]'); ylabel('position x [m]'); grid on;

figure; plot(t_out, x_out(:,3));
xlabel('time [s]'); ylabel('pressure p [Pa]'); grid on;
