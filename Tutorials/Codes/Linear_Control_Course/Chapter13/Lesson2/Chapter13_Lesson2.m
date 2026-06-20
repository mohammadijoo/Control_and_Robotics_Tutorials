% Parameters
J = 0.01;   % inertia
B = 0.1;    % viscous friction
K = 0.5;    % gain

s = tf('s');
G = K / (J * s + B);   % Transfer function object

% Frequency grid (rad/s)
w = logspace(-1, 3, 300);

% Frequency response
[mag, phase, wout] = bode(G, w);   % mag, phase are 3-D arrays
mag = squeeze(mag);                % convert to column vectors
phase = squeeze(phase);

% Example: display a few points
for k = 1:50:length(wout)
    fprintf('w = %7.3f rad/s, |G(jw)| = %7.3f, phase = %7.3f deg\n', ...
            wout(k), mag(k), phase(k));
end
