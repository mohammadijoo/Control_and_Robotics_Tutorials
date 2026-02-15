
function z = whole_body_qp_step(J1, b1, J2, b2, Aeq, beq, lb, ub)
    m = size(J1, 2);
    W1 = eye(size(J1, 1));
    W2 = eye(size(J2, 1));
    alpha1 = 1e3;
    alpha2 = 1.0;
    lambda_reg = 1e-6;

    H = alpha1 * (J1' * W1 * J1) + ...
        alpha2 * (J2' * W2 * J2) + ...
        lambda_reg * eye(m);

    g = -(alpha1 * (J1' * W1 * b1) + ...
          alpha2 * (J2' * W2 * b2));

    options = optimoptions('quadprog', ...
                           'Algorithm','interior-point-convex', ...
                           'Display','off');

    z = quadprog(H, g, [], [], Aeq, beq, lb, ub, [], options);
end
