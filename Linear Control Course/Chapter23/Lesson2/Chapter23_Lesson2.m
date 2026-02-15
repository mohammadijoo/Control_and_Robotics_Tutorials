J0 = 0.01;  b0 = 0.05;  k0 = 2.0;
rel_unc = 0.2;
N = 200;

poles_all = [];

for i = 1:N
    J = J0 * (1 + rel_unc*(2*rand - 1));
    b = b0 * (1 + rel_unc*(2*rand - 1));
    k = k0 * (1 + rel_unc*(2*rand - 1));

    sys = tf(1, [J b k]);   % G(s) = 1/(J s^2 + b s + k)
    poles_all = [poles_all; pole(sys).'];
end

figure; plot(real(poles_all), imag(poles_all), 'x');
xlabel('Re(s)'); ylabel('Im(s)');
title('Pole cloud under parametric uncertainty'); grid on;

% Nominal step response (e.g., joint angle under torque command)
sys_nom = tf(1, [J0 b0 k0]);
figure; step(sys_nom);
title('Nominal joint step response');
