
function ok = planar_constraint_check(q)
    % q: [q1; q2]
    l1 = 1.0; l2 = 0.7;
    q1_min = -pi/2; q1_max = pi/2;
    q2_min = -pi;   q2_max = pi;
    x_obs = 0.7; y_obs = 0.3; r_obs = 0.2;

    q1 = q(1); q2 = q(2);
    % Joint limits
    if q1 < q1_min || q1 > q1_max || q2 < q2_min || q2 > q2_max
        ok = false; return;
    end

    % FK
    x = l1*cos(q1) + l2*cos(q1 + q2);
    y = l1*sin(q1) + l2*sin(q1 + q2);

    % Obstacle constraint
    dx = x - x_obs; dy = y - y_obs;
    h = dx*dx + dy*dy - r_obs^2;  % should be >= 0
    if h < 0
        ok = false; return;
    end

    % Ground constraint y >= 0
    if y < 0
        ok = false; return;
    end

    ok = true;
end
