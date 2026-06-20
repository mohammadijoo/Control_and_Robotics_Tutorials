function lesson7_3_demo()
    % Planar 2R example: spatial and body Jacobians
    l1 = 1.0; l2 = 0.8;
    S1 = [0;0;1; 0;0;0];
    S2 = [0;0;1; 0;-l1;0];
    Slist = [S1, S2];

    M = eye(4);
    M(1,4) = l1 + l2;

    q = [0.5; -0.4];
    T = fk_poe_space(Slist, M, q);
    Js = jacobian_space(Slist, q);
    Jb = jacobian_body(Slist, M, q);

    disp('T(q) ='); disp(T);
    disp('J_s(q) ='); disp(Js);
    disp('J_b(q) ='); disp(Jb);
end

function S_hat = se3_hat(S)
    omega = S(1:3);
    v = S(4:6);
    S_hat = zeros(4,4);
    S_hat(1:3,1:3) = skew(omega);
    S_hat(1:3,4) = v;
end

function Sx = skew(v)
    vx = v(1); vy = v(2); vz = v(3);
    Sx = [ 0,  -vz,  vy;
          vz,   0,  -vx;
         -vy,  vx,   0];
end

function T = se3_exp(S, theta)
    omega = S(1:3);
    v = S(4:6);
    wnorm = norm(omega);
    if wnorm < 1e-9
        T = eye(4);
        T(1:3,4) = v * theta;
        return;
    end
    w = omega / wnorm;
    w_hat = skew(w);
    th = wnorm * theta;
    R = eye(3) + sin(th)*w_hat + (1-cos(th))*(w_hat*w_hat);
    G = eye(3)*th + (1-cos(th))*w_hat + (th - sin(th))*(w_hat*w_hat);
    p = G * (v/wnorm);
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p;
end

function Ad = adjoint(T)
    R = T(1:3,1:3);
    p = T(1:3,4);
    Ad = zeros(6,6);
    Ad(1:3,1:3) = R;
    Ad(4:6,4:6) = R;
    Ad(4:6,1:3) = skew(p)*R;
end

function T = fk_poe_space(Slist, M, q)
    T = eye(4);
    n = size(Slist,2);
    for i = 1:n
        T = T * se3_exp(Slist(:,i), q(i));
    end
    T = T * M;
end

function Js = jacobian_space(Slist, q)
    n = size(Slist,2);
    Js = zeros(6,n);
    T = eye(4);
    for i = 1:n
        if i == 1
            Js(:,1) = Slist(:,1);
        else
            AdT = adjoint(T);
            Js(:,i) = AdT * Slist(:,i);
        end
        T = T * se3_exp(Slist(:,i), q(i));
    end
end

function Jb = jacobian_body(Slist, M, q)
    T = fk_poe_space(Slist, M, q);
    Js = jacobian_space(Slist, q);
    AdInv = adjoint(inv(T));
    Jb = AdInv * Js;
end
      
