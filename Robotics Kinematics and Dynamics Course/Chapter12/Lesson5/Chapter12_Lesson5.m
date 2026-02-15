function lesson12_5_validation_matlab()
    p.m1 = 2.0; p.m2 = 1.0;
    p.l1 = 1.0; p.c1 = 0.5; p.c2 = 0.5;
    p.I1 = 0.2; p.I2 = 0.1;
    p.g  = 9.81;

    q   = [0.3; -0.7];
    dq  = [0.1; -0.2];
    ddq = [0.5; 0.3];

    tauL  = tau_lagrange_2R(q, dq, ddq, p);
    % tauNE should be computed by your Newton-Euler function
    % tauNE = tau_newton_euler_2R(q, dq, ddq, p, [0; -p.g; 0]);

    disp('tau_L (Lagrange) =');
    disp(tauL);

    % Example of reconstructing M_NE and g_NE for testing:
    MNE = reconstruct_M_NE(q, p);
    gNE = reconstruct_g_NE(q, p);
    disp('M_NE(q) ='); disp(MNE);
    disp('g_NE(q) ='); disp(gNE);
end

function M = M_lagrange_2R(q, p)
    q2 = q(2);
    a  = p.I1 + p.I2 + p.m1 * p.c1^2 + p.m2 * (p.l1^2 + p.c2^2);
    b  = p.m2 * p.l1 * p.c2;
    d  = p.I2 + p.m2 * p.c2^2;
    c2 = cos(q2);

    M = [a + 2*b*c2, d + b*c2;
         d + b*c2,   d];
end

function h = h_lagrange_2R(q, dq, p)
    q2  = q(2);
    dq1 = dq(1);
    dq2 = dq(2);
    b   = p.m2 * p.l1 * p.c2;
    s2  = sin(q2);

    h = [-b * s2 * (2*dq1*dq2 + dq2^2);
          b * s2 * dq1^2];
end

function g = g_lagrange_2R(q, p)
    q1 = q(1); q2 = q(2);
    g1 = p.g * (p.c1 * p.m1 * cos(q1) ...
             + p.m2 * (p.l1 * cos(q1) + p.c2 * cos(q1 + q2)));
    g2 = p.g * (p.m2 * p.c2 * cos(q1 + q2));
    g  = [g1; g2];
end

function tau = tau_lagrange_2R(q, dq, ddq, p)
    M = M_lagrange_2R(q, p);
    h = h_lagrange_2R(q, dq, p);
    g = g_lagrange_2R(q, p);
    tau = M * ddq + h + g;
end

function MNE = reconstruct_M_NE(q, p)
    dq   = [0; 0];
    gBase = [0; 0; 0];
    MNE  = zeros(2,2);
    for j = 1:2
        ddq = zeros(2,1);
        ddq(j) = 1;
        tau = tau_newton_euler_2R(q, dq, ddq, p, gBase);
        MNE(:, j) = tau;
    end
end

function gNE = reconstruct_g_NE(q, p)
    dq   = [0; 0];
    ddq  = [0; 0];
    gBase = [0; -p.g; 0];
    gNE  = tau_newton_euler_2R(q, dq, ddq, p, gBase);
end
      
