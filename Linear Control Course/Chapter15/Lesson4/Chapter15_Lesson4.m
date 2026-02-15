% Unstable plant with RHP pole and time delay
s = tf('s');
p = 1.0;
K = 2.0;
G0 = 1 / (s - p);
L_delay = 0.05;

% First-order Pade approximation of the delay
[numd, dend] = pade(L_delay, 1);
Delay = tf(numd, dend);

Lsys = K * G0 * Delay;

figure;
nyquist(Lsys);
title('Nyquist: unstable plant with time delay');

[gm, pm, wg, wp] = margin(Lsys);
fprintf('Gain margin: %g, phase margin: %g deg\n', gm, pm);
if wp > 0 && pm > 0
    Lmax = (pm * pi / 180) / wp;
    fprintf('Approximate delay margin L_max ≈ %g s\n', Lmax);
end

% Simulink hint:
%  - Use a "Transfer Fcn" block for 1/(s - p)
%  - Use a "Transport Delay" block with delay L_delay
%  - Close the loop with a "PID Controller" or "Gain" block
%  - Robotic models from Robotics System Toolbox can be linearized
%    around equilibria and connected to this loop for joint-level analysis.
