function stewart_demo()
    % Base and platform geometry (example values)
    B = [ 0.5, -0.5, 0.0;
          0.5,  0.5, 0.0;
          0.0,  0.6, 0.0;
         -0.5,  0.5, 0.0;
         -0.5, -0.5, 0.0;
          0.0, -0.6, 0.0 ];  % 6x3

    P = [ 0.3, -0.3, 0.0;
          0.3,  0.3, 0.0;
          0.0,  0.4, 0.0;
         -0.3,  0.3, 0.0;
         -0.3, -0.3, 0.0;
          0.0, -0.4, 0.0 ];  % 6x3

    % pose = [px, py, pz, phi, theta, psi]
    pose = [0.0, 0.0, 0.8, 0.0, 0.0, 0.0];

    L = stewart_inverse_kin(pose, B, P);
    JL = stewart_jacobian(pose, B, P);

    disp('Leg lengths:');
    disp(L.');
    disp('Length-rate Jacobian JL:');
    disp(JL);
end

function R = rotZYX(phi, theta, psi)
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    Rx = [1, 0, 0;
          0, cphi, -sphi;
          0, sphi, cphi];
    Ry = [cth, 0, sth;
          0, 1, 0;
          -sth, 0, cth];
    Rz = [cpsi, -spsi, 0;
          spsi,  cpsi, 0;
          0, 0, 1];
    R = Rz * Ry * Rx;
end

function L = stewart_inverse_kin(pose, B, P)
    px = pose(1); py = pose(2); pz = pose(3);
    phi = pose(4); theta = pose(5); psi = pose(6);

    p = [px; py; pz];
    R = rotZYX(phi, theta, psi);

    L = zeros(6, 1);
    for i = 1:6
        d = p + R * P(i, :).';
        d = d - B(i, :).';
        L(i) = norm(d);
    end
end

function JL = stewart_jacobian(pose, B, P)
    px = pose(1); py = pose(2); pz = pose(3);
    phi = pose(4); theta = pose(5); psi = pose(6);

    p = [px; py; pz];
    R = rotZYX(phi, theta, psi);

    JL = zeros(6, 6);
    for i = 1:6
        d = p + R * P(i, :).';
        d = d - B(i, :).';
        L = norm(d);
        u = d / L;
        Rp = R * P(i, :).';
        JL(i, 1:3) = u.';
        JL(i, 4:6) = cross(Rp, u).';
    end
end
      
