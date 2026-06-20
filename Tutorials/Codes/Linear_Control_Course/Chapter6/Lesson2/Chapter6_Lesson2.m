% Desired parameters for a robotic joint servo
zeta   = 0.7;
wn     = 8;           % rad/s
K_dc   = 1.0;         % DC gain

num = K_dc * wn^2;
den = [1  2*zeta*wn  wn^2];

G = tf(num, den);
disp('Transfer function G(s):');
G

p = pole(G)

% Recover parameters from poles (assuming stable system)
s1 = p(1);
s2 = p(2);

wn_hat   = sqrt(s1 * s2);
zeta_hat = - (s1 + s2) / (2 * wn_hat);

fprintf('Recovered zeta = %.4f, omega_n = %.4f\n', zeta_hat, wn_hat);

% Simulink: one can create a "Transfer Fcn" block with numerator [num]
% and denominator [den], then use "Step" blocks and scopes to study the
% time response within a larger robotic manipulator model.
