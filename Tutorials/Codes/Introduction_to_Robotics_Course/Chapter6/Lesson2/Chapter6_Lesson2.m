function demo_transmission()
    % Two-stage geartrain
    stages = [20 80; 18 54]; % each row: [z1 z2]
    N = prod(stages(:,2) ./ stages(:,1));

    fprintf('N_eq = %.3f\n', N);

    % Reflected inertia
    Jm = 2e-4; Jl = 5e-3;
    J_at_load  = Jl + (N^2)*Jm;
    J_at_motor = Jm + Jl/(N^2);

    fprintf('J_at_load = %.6f, J_at_motor = %.6f\n', J_at_load, J_at_motor);

    % Backlash torque curve
    b = 0.02; kt = 150;
    dtheta = linspace(-0.05, 0.05, 200);
    tauL = arrayfun(@(d) backlash_torque(N*d, 0, N, b, kt), dtheta);

    figure; plot(dtheta, tauL); grid on;
    xlabel('\Delta\theta'); ylabel('\tau_\ell');
    title('Backlash deadband torque model');
end

function tau = backlash_torque(theta_m, theta_l, N, b, kt)
    dtheta = theta_m/N - theta_l;
    if abs(dtheta) <= b/2
        tau = 0;
    elseif dtheta > b/2
        tau = kt*(dtheta - b/2);
    else
        tau = kt*(dtheta + b/2);
    end
end
