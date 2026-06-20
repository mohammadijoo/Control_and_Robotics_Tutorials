% Plant (same as in Python example)
J = 0.01; b = 0.1; k = 0.0;
G = tf(1, [J b k]);

% PD controller
Kp = 20; Kd = 0.5;
C = Kd * tf([1 0], 1) + Kp;

L = C * G;  % open-loop

% Measurement low-pass filter
wf = 50;   % rad/s
F = tf(wf, [1 wf]);

% Noise-to-output transfer functions
Gn0 = -L / (1 + L);        % no filter
GnF = -(L * F) / (1 + L * F);  % with filter

omega = logspace(0, 3, 400);
figure;
bodemag(Gn0, GnF, omega);
legend("no filter", "with measurement LPF");
grid on;
title("Noise-to-output frequency response");

% Simulink implementation:
%  - Use blocks: Sum, Transfer Fcn (G), PID Controller, Transfer Fcn (F),
%    Band-Limited White Noise for measurement noise.
%  - Connect noise source at sensor summing junction, place F in feedback path,
%    and scope the output to see the effect of filtering.
