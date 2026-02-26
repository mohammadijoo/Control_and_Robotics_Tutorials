% Characteristic polynomial P(s,K) = s^3 + 5 s^2 + 6 s + K
K = 10;
den = [1 5 6 K];

% Degree of stability from poles
poles = roots(den);
if any(real(poles) >= 0)
    alpha = 0;
else
    alpha = min(-real(poles));
end
disp(['Stable? alpha = ', num2str(alpha)]);

% Time-domain step response (for some nominal plant)
num = [K];   % assume G(s) = K / (s^3 + 5 s^2 + 6 s)
sys = tf(num, den);
step(sys);
grid on;

% Robotics context:
% In Robotics System Toolbox, a rigidBodyTree model can be linearized around
% an operating point to obtain A,B matrices. From that, the characteristic
% polynomial of a joint loop can be formed and analyzed similarly:
%
% [A,B,C,D] = linmod('robot_joint_simulink_model');
% poles = eig(A);
% alpha = min(-real(poles));
