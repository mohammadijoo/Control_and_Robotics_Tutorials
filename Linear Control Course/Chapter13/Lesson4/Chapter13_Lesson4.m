zeta = 0.6;
wn   = 13.5;  % rad/s

num = [wn^2];
den = [1 2*zeta*wn wn^2];
T = tf(num, den);

% Time-domain: step response
t = 0:0.001:2;
[y, t] = step(T, t);
y_final = y(end);
peak = max(y);
Mp = (peak - y_final)/y_final;

idx_out = find(abs(y - y_final) > 0.02*abs(y_final));
ts = t(idx_out(end));

fprintf('Mp = %.2f%%\n', Mp*100);
fprintf('ts = %.3f s\n', ts);

% Frequency-domain: frequency response
w = logspace(-1, 2, 500);  % rad/s
[mag, phase] = bode(T, w);
mag = squeeze(mag);

[Mr, idxMr] = max(mag);
wr = w(idxMr);

target = 1/sqrt(2);
idxBw = find(mag <= target, 1, 'first');
wB = w(idxBw);

fprintf('Mr = %.3f, wr = %.3f rad/s, wB = %.3f rad/s\n', Mr, wr, wB);

% For visualization:
% figure; step(T);
% figure; bode(T); grid on;  % Bode plots (dB and deg vs log w)
