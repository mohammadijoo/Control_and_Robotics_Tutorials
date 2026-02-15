% Second-order benchmark plant
zeta = 0.5;
wn   = 4.0;

num = [wn^2];              % numerator coefficients
den = [1  2*zeta*wn  wn^2];% denominator coefficients

G = tf(num, den);

% Time-domain analysis
figure;
step(G);
title('Step response of second-order plant');

% Frequency-domain analysis
figure;
bode(G);
grid on;
title('Bode plot of second-order plant');
