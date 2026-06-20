% Nominal parameters
J0 = 0.01;   % kg m^2
b0 = 0.05;   % N m s/rad

s = tf('s');
G0 = 1 / (J0*s^2 + b0*s);  % nominal joint model

% Parameter sweep (±20%)
nSamples = 6;
Jvals = J0 * (1 + 0.4*(rand(1,nSamples)-0.5));
bvals = b0 * (1 + 0.4*(rand(1,nSamples)-0.5));

t = linspace(0,4,500);
figure; hold on;
for k = 1:nSamples
    J = Jvals(k);
    b = bvals(k);
    G = 1 / (J*s^2 + b*s);
    step(G, t);
end
step(G0, t);  % nominal response
grid on;
title('Step responses with parametric uncertainty');
xlabel('Time [s]');
ylabel('Joint angle [rad]');

% Simulink hint:
% 1. Build a block diagram with Transfer Fcn block 1/(J*s^2 + b*s).
% 2. Use mask parameters for J and b and a "Model Variants" or parameter
%    sweep script to simulate different uncertainty samples.
% 3. Connect to a manipulator model from Robotics System Toolbox by
%    connecting joint torques and positions.
