function lesson16_3RPR_demo()
    % Base anchors
    B = [0.0, 0.0;
         1.0, 0.0;
         0.5, sqrt(3.0) / 2.0];

    % Platform anchors
    rp = 0.2;
    P = rp * [ 1.0,              0.0;
              -0.5,              sqrt(3.0) / 2.0;
              -0.5,             -sqrt(3.0) / 2.0];

    x_true  = 0.2;
    y_true  = 0.1;
    phi_true = 0.3;

    L = ik_3rpr_matlab([x_true; y_true; phi_true], B, P);

    z0 = [0.0; 0.0; 0.0];
    opts = optimoptions("fsolve", "Display", "off", "Jacobian", "on");
    [z_sol, fval, exitflag] = fsolve(@(z) fk_3rpr_matlab(z, L, B, P), z0, opts);

    disp("Exit flag:");
    disp(exitflag);
    disp("True pose:");
    disp([x_true; y_true; phi_true]);
    disp("FK pose:");
    disp(z_sol);
end

function L = ik_3rpr_matlab(z, B, P)
    x   = z(1);
    y   = z(2);
    phi = z(3);
    R = [cos(phi), -sin(phi);
         sin(phi),  cos(phi)];
    t = [x; y];
    L = zeros(3, 1);
    for i = 1:3
        ci = t + R * P(i, :).';   % platform anchor in base frame
        pi = ci - B(i, :).';
        L(i) = norm(pi);
    end
end

function [Phi, J] = fk_3rpr_matlab(z, L, B, P)
    x   = z(1);
    y   = z(2);
    phi = z(3);

    c = cos(phi);
    s = sin(phi);
    R = [c, -s;
         s,  c];
    Rphi = [-s, -c;
             c, -s];

    t = [x; y];
    Phi = zeros(3, 1);
    J = zeros(3, 3);

    for i = 1:3
        Pi = P(i, :).';
        ci = t + R * Pi;
        pi = ci - B(i, :).';
        Phi(i) = pi.' * pi - L(i)^2;

        dpi_dphi = Rphi * Pi;
        J(i, 1) = 2.0 * pi(1);
        J(i, 2) = 2.0 * pi(2);
        J(i, 3) = 2.0 * (pi.' * dpi_dphi);
    end
end
      
