% Plant: G(s) = K / (s (0.1 s + 1))
K = 5;
s = tf('s');
G = K / (s * (0.1*s + 1));

% Unity feedback
T = feedback(G, 1);

% Nichols plot of open-loop
figure;
nichols(G);
grid on;
title('Open-loop Nichols locus with M and N contours');

% Closed-loop Bode from Nichols (conceptual: MATLAB internally uses T(s))
figure;
bode(T);
grid on;
title('Closed-loop Bode plot from T(s) = G(s)/(1+G(s))');

% Resonant peak and bandwidth from closed-loop Bode
[Mr, wMr] = getPeakGain(T);      % closed-loop resonant peak
[mag, phase, w] = bode(T);
mag = squeeze(mag);
mag_db = 20*log10(mag);
idx = find(mag_db >= -3, 1, 'last'); % approximate bandwidth
wbw = w(idx);

fprintf('Closed-loop peak Mr = %.3f (%.2f dB) at w = %.3f rad/s\n', ...
        Mr, 20*log10(Mr), wMr);
fprintf('Approximate closed-loop bandwidth ~ %.3f rad/s\n', wbw);

% In Simulink, this plant and controller could be placed in a feedback loop;
% the Nichols chart guides controller tuning for desired Mr and bandwidth.
