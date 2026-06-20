% Controller gains
Kp = 3.2;
Ki = 9.0;

% Continuous-time transfer functions (Control System Toolbox)
s = tf('s');
G = 1 / (s + 1);
C = Kp + Ki / s;

% Closed-loop with unity feedback and optional reference filter F(s)
F = 1;        % or F = wf / (s + wf) for some wf < 3
T_r = feedback(F * C * G, 1);   % reference-to-output
T_d = feedback(G, C);           % disturbance-to-output (input disturbance)

% Time vector and signals
t = 0:0.001:8;
r = ones(size(t));              % unit step
d = zeros(size(t));
d(t >= 3) = 0.2;                % step disturbance

% Simulate reference tracking only
[y_r, t_r] = step(T_r, t(end));

% Simulate combined reference and disturbance via lsim
sys_cl = ss([0 0; -1-Kp Ki; 1 0], [1 0; Kp Ki; 0 0], [0 1 0], 0);
% State vector: [z; y], input vector: [r; d]
u_input = [r.' d.'];
[y_cl, t_cl] = lsim(sys_cl, u_input, t);

plot(t_cl, y_cl), grid on
xlabel('Time [s]'), ylabel('Output y(t)')
title('PI tracking with disturbance step at t = 3 s')

% In Simulink, you would build an equivalent diagram using:
%   - Sum blocks for error and disturbance
%   - Discrete or continuous PI controller block
%   - First-order Transfer Fcn block for G(s)
%   - Optional prefilter F(s) in the reference path.
