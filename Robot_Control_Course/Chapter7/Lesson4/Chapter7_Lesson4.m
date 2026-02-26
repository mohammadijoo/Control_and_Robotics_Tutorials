
% Parameters
J_nom = 1.0;      % nominal inertia
wc    = 50;       % DOB cutoff frequency (rad/s)

s = tf('s');

% Nominal joint model and inverse
Pn   = 1 / (J_nom * s^2);
Pinv = J_nom * s^2;

% Q-filter
Q = wc / (s + wc);

% Example: disturbance observer block in transfer function form
% d_hat(s) = Q(s) * (Pinv(s) * y(s) - tau_c(s))

% In Simulink:
% 1) Use a second derivative block (or 'Derivative' plus filtering) on q(t)
% 2) Multiply by J_nom to get J_nom * q_ddot
% 3) Subtract tau_c to form J_nom * q_ddot - tau_c
% 4) Pass through first-order transfer function wc / (s + wc)
% 5) Subtract d_hat from tau_c to obtain tau = tau_c - d_hat
