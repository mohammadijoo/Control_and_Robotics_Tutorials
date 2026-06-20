wn   = 10;     % natural frequency
zeta = 0.4;    % damping ratio
K    = 2;      % proportional gain

s = tf('s');

G = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);  % plant
C = K;                                   % controller

L = C*G;
T = feedback(L, 1);                      % closed-loop

% Bode plot with margins
figure;
margin(L); grid on;
title('Loop transfer function L(s) with stability margins');

[gm, pm, w_pc, w_gc] = margin(L);
fprintf('Gain margin (dB): %g\n', 20*log10(gm));
fprintf('Phase margin (deg): %g\n', pm);
fprintf('w_gc = %g rad/s, w_pc = %g rad/s\n', w_gc, w_pc);

% Closed-loop bandwidth
figure;
bode(T); grid on;
title('Closed-loop transfer function T(s)');

% Estimate -3 dB bandwidth
[magT, phaseT, w] = bode(T);
magT = squeeze(magT);
w = squeeze(w);
mag0 = magT(1);
idx = find(magT < mag0/sqrt(2), 1, 'first');
if ~isempty(idx)
    wb = w(idx);
    fprintf('Approx. bandwidth wb = %g rad/s\n', wb);
    fprintf('Approx. settling time ts ~ 4/wb = %g s\n', 4/wb);
end

% Robotics note:
% For a robot joint model, you might derive a state-space model using
% robotics toolbox functions, then convert to tf and repeat the same steps.

% Example Simulink usage:
% - Build a unity feedback loop with C(s) and G(s) blocks.
% - Use the Linear Analysis Tool to linearize the model about an
%   operating point and generate Bode and Nyquist plots automatically.
