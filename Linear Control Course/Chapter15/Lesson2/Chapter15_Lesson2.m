% Loop transfer function for a robot joint position servo
% G(s) = 1 / ((s+0.1)(s+1)), C(s) = K
K = 5;
s = tf('s');
G = 1 / ((s + 0.1) * (s + 1));
C = K;
L = C * G;

% Nyquist plot
figure;
nyquist(L);
grid on;
title('Nyquist plot of L(s) = C(s)G(s)');

% Nyquist-based closed-loop stability (open-loop stable, P=0)
% For unity feedback, use feedback(L,1) or feedback(C*G,1)
T = feedback(L, 1);
disp('Closed-loop poles:');
disp(pole(T));

if all(real(pole(T)) < 0)
    disp('Closed loop is asymptotically stable.');
else
    disp('Closed loop is unstable (some poles in RHP or on jw axis).');
end
