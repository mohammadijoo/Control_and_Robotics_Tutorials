% Desired specs (e.g., for a robot joint position loop)
Mp_max = 0.1;   % 10% max overshoot
ts_max = 1.0;   % 1 s settling time (2% criterion)

% Approximate mapping Mp -> zeta (could be refined via fsolve)
zeta = 0.6;  % approx for 10% Mp

wn   = 4 / (zeta * ts_max);

s = tf('s');
T = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

% Time-domain analysis
info = stepinfo(T);
fprintf('Mp    = %.3f\n', info.Overshoot/100);
fprintf('ts    = %.3f s\n', info.SettlingTime);
fprintf('tp    = %.3f s\n', info.PeakTime);

% Frequency-domain analysis
[mag, phase, w] = bode(T);
mag = squeeze(mag);
mag_db = 20*log10(mag);
[~, idx] = min(abs(mag_db + 3));
omega_B = w(idx);

fprintf('omega_B ~ %.3f rad/s\n', omega_B);

% Plot for inspection
figure;
subplot(2,1,1);
step(T);
grid on; title('Closed-loop step response');

subplot(2,1,2);
bodemag(T);
grid on; title('Closed-loop magnitude response');

% In Simulink, T can be realized with Transfer Fcn blocks,
% and connected in a unity-feedback loop around a plant model
% for robot joint dynamics.
