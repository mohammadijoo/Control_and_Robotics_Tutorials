% Define transfer function G(s) = 1 / (s^2 + 3 s + 2)
num = 1;
den = [1 3 2];
G = tf(num, den);

% Poles and stability check
p = pole(G);
isAsymptoticallyStable = all(real(p) < 0);

disp('Poles of G(s):');
disp(p);

if isAsymptoticallyStable
    disp('System is asymptotically stable.');
else
    disp('System is NOT asymptotically stable.');
end

% Time response check (e.g., step response)
figure;
step(G);
grid on;
title('Step response of G(s) = 1 / (s^2 + 3 s + 2)');
