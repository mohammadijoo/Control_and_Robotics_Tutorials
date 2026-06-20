% Parameters for a simple robot joint model
J = 0.01;
b = 0.1;

% Transfer function from torque to angle
num = 1;
den = [J, b, 0];  % J s^2 + b s

G = tf(num, den);
disp('Open-loop poles:');
p_open = pole(G)

% Simple proportional position controller
Kp = 5;
G_cl = feedback(Kp * G, 1);
disp('Closed-loop poles:');
p_cl = pole(G_cl)

% Stability classification (vectorized logic)
if any(real(p_cl) > 0)
    disp('Closed-loop system is unstable');
elseif any(abs(real(p_cl)) < 1e-6)
    disp('Closed-loop system is marginally stable (not asymptotically stable)');
else
    disp('Closed-loop system is asymptotically stable');
end

% State-space form and eigenvalues
[A, B, C, D] = ssdata(G_cl);
lambda = eig(A)

% Simulink note:
% A Simulink model of the robot joint (with actuator, sensor, and controller)
% can be linearized around an operating point using:
% sys_lin = linearize('robot_joint_model', io);
% pole(sys_lin)
