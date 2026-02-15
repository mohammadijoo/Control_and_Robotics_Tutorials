% obs: N x d feature matrix (encoded deformable states)
% u_star: N x m expert actions (e.g., Cartesian velocities)

% Solve W in least squares sense: minimize ||u_star - obs * W'||_F^2
W = (obs' * obs) \ (obs' * u_star);  % d x m

% Save weights for Simulink
save("policy_weights.mat", "W");

% In Simulink:
%  1. Create an input block that outputs obs_t (1 x d).
%  2. Use a "MATLAB Function" or "Gain" block that implements u_t = obs_t * W.
%  3. Connect u_t to the robot dynamics block (or joint-level controller).
      
