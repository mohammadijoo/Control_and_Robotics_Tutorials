% Plant and variable s
s = tf('s');
G = 100 / (s * (0.1 * s + 1));

% --- Case Study 1: proportional control ---
Kc1 = 0.05;
C1 = Kc1;
L1 = C1 * G;
T1 = feedback(L1, 1);

% --- Case Study 2: lead compensator ---
Kc2 = 0.2;
C_lead = Kc2 * (s + 10) / (s + 20);
L2 = C_lead * G;
T2 = feedback(L2, 1);

% Root locus of compensated system
figure;
rlocus(L2);
title('Root locus of lead-compensated loop');

% Step responses
t = 0:0.002:2;
[y1, t1] = step(T1, t);
[y2, t2] = step(T2, t);

figure;
plot(t1, y1, 'LineWidth', 1.5); hold on;
plot(t2, y2, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Joint angle response');
legend('Case 1: Kc = 0.05', 'Case 2: Lead C_l(s)');

% Simulink implementation:
%  - Use a Transfer Fcn block for G(s).
%  - Implement C1(s) = Kc1 as a Gain block, or C_l(s) as a series of
%    Zero-Pole blocks (zero at -10, pole at -20) plus a Gain block 0.2.
%  - Close the unity feedback loop around the joint and apply step inputs
%    to observe tracking performance.
