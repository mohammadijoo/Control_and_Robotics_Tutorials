% Parameters
tau = 0.05;
wc  = 1 / tau;
w0  = 50;
zeta = 0.05;
Q_notch = 10;

s = tf('s');

% Low-pass and high-pass
H_LP = wc / (s + wc);
H_HP = s / (s + wc);

% Band-pass
H_BP = 2*zeta*w0*s / (s^2 + 2*zeta*w0*s + w0^2);

% Notch
H_N  = (s^2 + w0^2) / (s^2 + (w0/Q_notch)*s + w0^2);

% Bode plots
w = logspace(-1, 3, 600);
bode(H_LP, H_HP, H_BP, H_N, w);
legend('LP','HP','BP','Notch');
grid on;

% Robotics context: filter noisy joint-velocity estimate
% Suppose Gp is a joint plant model and C is a PID controller:
% L = C * H_LP * Gp;  % loop transfer function including measurement low-pass
