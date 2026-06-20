function so3_demo
    theta = deg2rad(30);

    Rz = rotz_local(theta);
    Ry = roty_local(theta);
    R  = Rz * Ry;

    I  = eye(3);
    orthErr = norm(R' * R - I, "fro");
    detR    = det(R);

    fprintf("R =\n");
    disp(R);
    fprintf("orthErr = %g\n", orthErr);
    fprintf("det(R)   = %g\n", detR);

    if orthErr < 1e-9 && abs(detR - 1.0) < 1e-9
        fprintf("R is numerically in SO(3).\n");
    else
        fprintf("R fails SO(3) test.\n");
    end
end

function R = rotx_local(theta)
    c = cos(theta); s = sin(theta);
    R = [1, 0, 0;
         0,  c, -s;
         0,  s,  c];
end

function R = roty_local(theta)
    c = cos(theta); s = sin(theta);
    R = [ c, 0,  s;
          0, 1,  0;
         -s, 0,  c];
end

function R = rotz_local(theta)
    c = cos(theta); s = sin(theta);
    R = [ c, -s, 0;
          s,  c, 0;
          0,  0, 1];
end
      
