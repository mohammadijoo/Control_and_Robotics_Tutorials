function demo_se3_exp_log()
    omega = [0; 0; 1];
    theta = pi/4;
    v = [0.1; 0.0; 0.0];
    xi = [v; omega];

    T = se3_exp(xi, theta);
    [xi_rec, theta_rec] = se3_log(T);

    disp('T ='); disp(T);
    disp('Recovered xi and theta:');
    disp(xi_rec.'); disp(theta_rec);
end

function W = hat_omega(omega)
    wx = omega(1); wy = omega(2); wz = omega(3);
    W = [  0,   -wz,   wy;
          wz,    0,   -wx;
         -wy,   wx,    0 ];
end

function [omega, theta] = so3_log(R)
    cos_theta = (trace(R) - 1) / 2;
    cos_theta = max(-1, min(1, cos_theta));
    theta = acos(cos_theta);
    if theta < 1e-9
        omega = [0; 0; 0];
        theta = 0;
        return;
    end
    W = (theta / (2 * sin(theta))) * (R - R');
    omega = [W(3, 2); W(1, 3); W(2, 1)];
    omega = omega / norm(omega);
end

function R = so3_exp(omega, theta)
    n = norm(omega);
    if n < 1e-9
        R = eye(3);
        return;
    end
    omega = omega / n;
    W = hat_omega(omega);
    W2 = W * W;
    R = eye(3) + sin(theta) * W + (1 - cos(theta)) * W2;
end

function T = se3_exp(xi, theta)
    v = xi(1:3);
    omega = xi(4:6);
    T = eye(4);
    if norm(omega) < 1e-9
        % pure translation
        T(1:3, 4) = v * theta;
        return;
    end
    omega = omega / norm(omega);
    W = hat_omega(omega);
    W2 = W * W;
    R = so3_exp(omega, theta);
    I3 = eye(3);
    J = I3 * theta + (1 - cos(theta)) * W + (theta - sin(theta)) * W2;
    p = J * v;
    T(1:3, 1:3) = R;
    T(1:3, 4) = p;
end

function [xi, theta] = se3_log(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    [omega, theta] = so3_log(R);
    if theta < 1e-9
        theta = norm(p);
        if theta < 1e-9
            xi = zeros(6, 1);
            theta = 0;
            return;
        end
        v = p / theta;
        xi = [v; 0; 0; 0];
        return;
    end
    W = hat_omega(omega);
    W2 = W * W;
    I3 = eye(3);
    Jinv = I3 ...
           - 0.5 * W ...
           + (1 / (theta^2) - (1 + cos(theta)) / (2 * theta * sin(theta))) * W2;
    v = Jinv * p;
    xi = [v; omega];
end
      
