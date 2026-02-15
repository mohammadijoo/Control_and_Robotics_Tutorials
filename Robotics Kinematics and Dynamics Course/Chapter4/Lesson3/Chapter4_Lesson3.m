function demo_open_closed_kinematics()
    % Open-chain 2R forward kinematics
    L = [0.5; 0.4];
    q = [0.5; -0.3];
    p_ee = fk_planar_2R(q, L);
    fprintf('Open-chain end-effector: (%.4f, %.4f)\n', p_ee(1), p_ee(2));

    % Closed-chain four-bar: solve for dependent angle
    L0 = 0.8; L1 = 0.5; L2 = 0.7; L3 = 0.4;
    theta1 = 0.4;
    theta3_init = 0.2;
    theta3 = solve_fourbar_theta3(theta1, L0, L1, L2, L3, theta3_init);
    Phi_res = fourbar_constraint(theta1, theta3, L0, L1, L2, L3);
    fprintf('Closed-chain theta3: %.6f, residual: %.3e\n', theta3, Phi_res);
end

function p2 = fk_planar_2R(q, L)
    theta1 = q(1); theta2 = q(2);
    L1 = L(1); L2 = L(2);
    R1 = rot2(theta1);
    R12 = rot2(theta1 + theta2);
    p1 = R1 * [L1; 0];
    p2 = p1 + R12 * [L2; 0];
end

function R = rot2(theta)
    c = cos(theta); s = sin(theta);
    R = [c, -s; s, c];
end

function Phi = fourbar_constraint(theta1, theta3, L0, L1, L2, L3)
    pB = [L1 * cos(theta1); L1 * sin(theta1)];
    pC = [L0 + L3 * cos(theta3); L3 * sin(theta3)];
    d = pB - pC;
    Phi = d' * d - L2^2;
end

function theta3 = solve_fourbar_theta3(theta1, L0, L1, L2, L3, theta3_init)
    % Use fsolve (Optimization Toolbox) if available; otherwise, simple Newton.
    theta3 = theta3_init;
    for k = 1:50
        Phi = fourbar_constraint(theta1, theta3, L0, L1, L2, L3);
        h = 1e-6;
        Phi_p = fourbar_constraint(theta1, theta3 + h, L0, L1, L2, L3);
        Phi_m = fourbar_constraint(theta1, theta3 - h, L0, L1, L2, L3);
        dPhi = (Phi_p - Phi_m) / (2*h);
        if abs(dPhi) < 1e-14
            break;
        end
        step = Phi / dPhi;
        theta3 = theta3 - step;
        if abs(step) < 1e-10
            break;
        end
    end
end
      
