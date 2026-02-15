% Robot joint parameters
J  = 0.01;
b  = 0.1;
Km = 0.5;

s = tf('s');
G = Km / (J*s^2 + b*s);   % G(s) = Km / (J s^2 + b s)

% Desired specs
zeta_min = 0.6;
Ts_max   = 1.0;

% Initial lead compensator guess
zc = 10;   % zero at s = -zc
pc = 50;   % pole at s = -pc
C  = (s + zc) / (s + pc);

L = G * C;

figure;
rlocus(L);
hold on;

% Overlay grid of damping and natural frequency
zeta_list = [zeta_min];
wn_list   = 0.1:0.1:50;
sgrid(zeta_list, wn_list);

% Vertical line for Ts_max: sigma = -4 / Ts_max
sigma_min = -4 / Ts_max;
plot([sigma_min sigma_min], ylim, 'k--');

title('Root locus with lead compensator');

% Interactively pick desired closed-loop pole on root locus
disp('Click on the root locus at desired pole location...');
[K_star, poles_star] = rlocfind(L);

fprintf('Selected gain K = %.3f\n', K_star);
disp('Dominant poles:');
disp(poles_star);

T = feedback(K_star * L, 1);
figure;
step(T);
grid on;
title('Step response with shaped root locus');
