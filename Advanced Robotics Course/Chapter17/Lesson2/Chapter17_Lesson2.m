function demo_rope_state()
    n = 8;              % number of nodes
    m = 0.05;           % mass per node
    k_spring = 20.0;    % spring stiffness
    x0 = zeros(6*n, 1); % initial state [q; qdot]

    % Mass matrix M and stiffness K for 1D chain, lifted to 3D
    M_scalar = m * eye(n);
    L = zeros(n, n);
    for i = 1:(n-1)
        L(i, i)     = L(i, i) + 1;
        L(i+1, i+1) = L(i+1, i+1) + 1;
        L(i, i+1)   = L(i, i+1) - 1;
        L(i+1, i)   = L(i+1, i) - 1;
    end
    K_scalar = k_spring * L;

    M = kron(M_scalar, eye(3));
    K = kron(K_scalar, eye(3));

    % Continuous-time state-space: xdot = [qdot; -M^{-1} K q + M^{-1} f_ext]
    % In Simulink, implement this as a custom block.
    A = [zeros(3*n) eye(3*n); -M\K zeros(3*n)];
    B = [zeros(3*n, 3*n); M\eye(3*n)];  % external forces as input
    C = eye(6*n);
    D = zeros(6*n, 3*n);

    sys = ss(A, B, C, D);
    disp(sys);
end
      
