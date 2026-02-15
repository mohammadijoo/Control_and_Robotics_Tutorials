wn   = 5.0;
zeta = 0.4;
alpha = 50.0;

% Full third-order transfer function
num_full = [wn^2];
den_full = conv([1 alpha], [1 2*zeta*wn wn^2]);
G3 = tf(num_full, den_full);

% Dominant second-order approximation
num_dom = [wn^2 / alpha];
den_dom = [1 2*zeta*wn wn^2];
G2 = tf(num_dom, den_dom);

% Step responses
t = linspace(0, 4, 1000);
[y_full, t1] = step(G3, t);
[y_dom,  t2] = step(G2, t);

figure;
plot(t1, y_full, t2, y_dom, '--');
grid on;
xlabel('Time (s)');
ylabel('Output');
legend('Full 3rd order', 'Dominant 2nd order');
title('Dominant Pole Approximation in MATLAB');

% Robotics context (sketch):
%   - Use Robotics System Toolbox to build a rigid-body tree for a manipulator
%   - Linearize a joint around an operating point to obtain G3(s)
%   - Reduce to G2(s) using dominant poles, then design a PID controller on G2(s)
