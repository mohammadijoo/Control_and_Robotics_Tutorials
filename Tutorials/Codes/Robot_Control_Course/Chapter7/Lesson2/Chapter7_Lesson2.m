
b   = 0.2;
k_p = 25.0;
k_d = 10.0;

J_values = linspace(0.5, 1.5, 21);
robustStable = true;

for J = J_values
    A = [0,              1;
        -k_p / J,  -(b + k_d) / J];
    lam = eig(A);
    if max(real(lam)) >= 0
        fprintf('Potential instability at J = %.3f\n', J);
        disp(lam);
        robustStable = false;
        break;
    end
end

fprintf('Robust stability on grid: %d\n', robustStable);

% Optional: connect to a Simulink model "single_joint_pd"
% where J is a workspace parameter used in the dynamics block.
% set_param('single_joint_pd', 'SimulationCommand', 'start');
