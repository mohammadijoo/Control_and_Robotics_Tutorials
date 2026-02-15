
% Toy Jacobian mapping in MATLAB
q  = [0.2; -0.4; 0.1];
J  = [1.0 0.2 0.0;
      0.0 1.0 0.3];
qd = [0.5; 0.0; -0.2];
yd = J*qd

% MATLAB ecosystems you will meet later:
% - Robotics System Toolbox (rigidBodyTree, sensors, planners)
% - Simulink for real-time control blocks
      