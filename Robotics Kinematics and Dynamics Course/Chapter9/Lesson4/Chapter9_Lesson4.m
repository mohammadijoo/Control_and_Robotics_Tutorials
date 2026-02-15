function [lambda_star, f_hat, limiting] = directionalCapacityBox2R(q1, q2, l1, l2, f_dir, tau_max)
%DIRECTIONALCAPACITYBOX2R  Planar 2R directional load capacity (static).
%
% Inputs:
%   q1, q2   Joint angles (rad)
%   l1, l2   Link lengths
%   f_dir    2x1 desired force direction (need not be unit length)
%   tau_max  2x1 positive torque limits [tau1_max; tau2_max]
%
% Outputs:
%   lambda_star  Maximum scalar lambda such that lambda * f_hat is feasible
%   f_hat        2x1 normalized direction vector
%   limiting     Indices of limiting joints at lambda_star

    eps = 1e-9;
    if norm(f_dir) < eps
        error('Direction vector must be nonzero.');
    end
    f_hat = f_dir / norm(f_dir);

    % Jacobian
    s1  = sin(q1);  c1  = cos(q1);
    s12 = sin(q1 + q2);  c12 = cos(q1 + q2);

    J = [ -l1*s1 - l2*s12, -l2*s12;
           l1*c1 + l2*c12,  l2*c12 ];

    v = J.' * f_hat;  % mapping to torques
    lambda_list = [];
    idx_list = [];

    for i = 1:2
        v_i = v(i);
        if abs(v_i) < eps
            continue;
        end
        lam_i = tau_max(i) / abs(v_i);
        lambda_list(end+1) = lam_i; %#ok<AGROW>
        idx_list(end+1) = i;        %#ok<AGROW>
    end

    if isempty(lambda_list)
        lambda_star = inf;
        limiting = [];
    else
        lambda_star = min(lambda_list);
        limiting = idx_list(abs(lambda_list - lambda_star) < 1e-6);
    end
end
      
