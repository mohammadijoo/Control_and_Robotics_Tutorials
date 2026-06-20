% Load features Phi_tilde (N x (d+1)) and actions A (N x m)
load("manip_demos.mat", "Phi_tilde", "A");  % variables: Phi_tilde, A

[N, d1] = size(Phi_tilde);
[~, m]  = size(A);
lambda  = 1e-3;

G   = Phi_tilde' * Phi_tilde + N * lambda * eye(d1);
RHS = Phi_tilde' * A;

W_tilde = G \ RHS;          % (d1 x m)
d = d1 - 1;
W = W_tilde(1:d, :);        % (d x m)
b = W_tilde(d1, :).';       % (m x 1)

% Save parameters for Simulink block
save("policy_params.mat", "W", "b");

% --- MATLAB Function block (inside Simulink) ---
% function a_cmd = imitation_policy(s)
% %#codegen
% load('policy_params.mat', 'W', 'b');
% a_cmd = W' * s + b;
      
