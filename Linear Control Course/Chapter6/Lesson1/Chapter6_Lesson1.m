wn   = 4.0;   % natural frequency (rad/s)
zeta = 0.7;   % damping ratio
K    = 1.0;   % DC gain

num = K * wn^2;
den = [1, 2*zeta*wn, wn^2];

G = tf(num, den);

figure;
step(G);
grid on;
title('Standard second-order step response (MATLAB)');
xlabel('time (s)');
ylabel('output y(t)');

% In Robotics System Toolbox, similar transfer functions are used as
% components inside manipulator and mobile robot controllers.
