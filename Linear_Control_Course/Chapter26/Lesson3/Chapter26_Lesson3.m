% Parameters
tau  = 0.02;          % time constant [s]
omega_c = 1 / tau;

% Continuous-time transfer function F(s) = omega_c / (s + omega_c)
s = tf('s');
F = omega_c / (s + omega_c);

% Example: true signal and noisy measurement
dt   = 0.001;
t    = 0:dt:2;
omega_signal = 2.0;
y_true = sin(omega_signal * t);
noise  = 0.05 * randn(size(t));
y_meas = y_true + noise;

% Filtered measurement via lsim
y_filt = lsim(F, y_meas, t);

% Plot for inspection
figure;
subplot(3,1,1); plot(t, y_true); ylabel('y true');
subplot(3,1,2); plot(t, y_meas); ylabel('y meas');
subplot(3,1,3); plot(t, y_filt); ylabel('y filt'); xlabel('t [s]');

% Simulink:
%  - Place a Transfer Fcn block with numerator [omega_c] and denominator [1 omega_c].
%  - Feed the sensor signal into the block; use the block output as the filtered measurement.
