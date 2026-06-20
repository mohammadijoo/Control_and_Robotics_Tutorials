function q_proj = projectPlanar2R(q_init, p_des, max_iters, tol)
    if nargin < 3, max_iters = 20; end
    if nargin < 4, tol = 1e-10; end
    L1 = 1.0; L2 = 1.0;
    q = q_init(:);
    for k = 1:max_iters
        h = fk_planar_2R(q, L1, L2) - p_des(:);
        J = jacobian_planar_2R(q, L1, L2);
        J_pinv = pinv(J);  % 2x2 pseudoinverse
        q = q - J_pinv * h;
        if norm(h) < tol
            break;
        end
    end
    q_proj = q;
end

function p = fk_planar_2R(q, L1, L2)
    th1 = q(1); th2 = q(2);
    x = L1 * cos(th1) + L2 * cos(th1 + th2);
    y = L1 * sin(th1) + L2 * sin(th1 + th2);
    p = [x; y];
end

function J = jacobian_planar_2R(q, L1, L2)
    th1 = q(1); th2 = q(2);
    s1  = sin(th1);
    c1  = cos(th1);
    s12 = sin(th1 + th2);
    c12 = cos(th1 + th2);
    J = [ -L1 * s1 - L2 * s12, -L2 * s12;
           L1 * c1 + L2 * c12,  L2 * c12 ];
end
      
