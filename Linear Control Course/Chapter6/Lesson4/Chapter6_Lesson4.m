% Parameters
zeta = 0.4;
wn   = 5.0;

% Baseline second-order
num0 = wn^2;
den0 = [1  2*zeta*wn  wn^2];
G0 = tf(num0, den0);

% Extra pole at s = -a
a  = 20;
kp = a;
num_p = kp * wn^2;
den_p = conv([1 a], den0);
Gp = tf(num_p, den_p);

% Extra zero at s = -z
z  = 2;
kz = 1/z;
num_z = [kz*wn^2  kz*z*wn^2];  % (s + z)*wn^2 / z
den_z = den0;
Gz = tf(num_z, den_z);

% Step responses
t = 0:0.001:4;
[y0, t0] = step(G0, t);
[yp, tp] = step(Gp, t);
[yz, tz] = step(Gz, t);

figure;
plot(t0, y0, 'LineWidth', 1.5); hold on;
plot(tp, yp, 'LineWidth', 1.5);
plot(tz, yz, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Response y(t)');
legend('Second-order', 'Extra pole', 'Extra zero');

title('Effect of Additional Pole and Zero on Step Response');

% -------------------------
% Simple Simulink setup (script-driven)
% -------------------------
% You can create a Simulink model programmatically:
%   new_system('extraPoleZeroModel');
%   open_system('extraPoleZeroModel');
% Then insert Transfer Fcn blocks with numerator/denominator corresponding
% to G0, Gp, and Gz, and connect them to a Step block and Scope blocks.
% Robotics System Toolbox can be used to connect these low-order models
% to manipulator or mobile-robot dynamics blocks.
