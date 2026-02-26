% Plant and compensator zero
s = tf('s');
G = 1 / (s * (s + 2));
zc = 8;              % zero at s = -8
Cz = (s + zc);       % C(s) = s + 8

% Open-loop without gain
Lbase = Cz * G;

% Desired dominant pole
zeta = 0.5;
wn = 4.0;
sd = -zeta * wn + 1j * wn * sqrt(1 - zeta^2);

% Gain from magnitude condition
Lval = evalfr(Lbase, sd);
K = 1 / abs(Lval);

% Closed-loop system
C = K * Cz;
T = feedback(C * G, 1);

% Visualizations
figure; rlocus(C * G); grid on; title('Root locus with real zero at -8');
figure; step(T); grid on; title('Closed-loop step response');

% Simulink notes:
% 1. In Simulink, place blocks for G(s) and C(s), and close the loop with unity feedback.
% 2. Use Transfer Fcn blocks for 1/(s(s+2)) and (s+8).
% 3. Parameter K can be implemented as a Gain block.
