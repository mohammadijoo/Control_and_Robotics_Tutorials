
function demo_safe_set_2d()
    q = [0.0; 0.0];
    q_min = [-1.0; -1.0];
    q_max = [ 1.0;  1.0];
    l1 = 0.8; l2 = 0.6;
    c = [0.8; 0.0];
    r_obs = 0.1;
    d_min = 0.05;
    margin = 0.0;
    tol = 0.0;

    h_joints = joint_limit_barrier(q, q_min, q_max, margin);
    h_obs = obstacle_barrier(q, l1, l2, c, r_obs, d_min);

    fprintf('h_joints = [%f %f %f %f]\n', h_joints);
    fprintf('h_obs = %f\n', h_obs);
    fprintf('Is safe? %d\n', is_safe(q, q_min, q_max, l1, l2, c, r_obs, d_min, margin, tol));
end

function p = fk_2d(q, l1, l2)
    q1 = q(1); q2 = q(2);
    x = l1 * cos(q1) + l2 * cos(q1 + q2);
    y = l1 * sin(q1) + l2 * sin(q1 + q2);
    p = [x; y];
end

function h = joint_limit_barrier(q, q_min, q_max, margin)
    h_low = q - (q_min + margin);
    h_up  = (q_max - margin) - q;
    h = [h_low; h_up];
end

function h = obstacle_barrier(q, l1, l2, c, r_obs, d_min)
    p = fk_2d(q, l1, l2);
    diff = p - c;
    dist2 = diff' * diff;
    rho = r_obs + d_min;
    h = dist2 - rho^2;
end

function safe = is_safe(q, q_min, q_max, l1, l2, c, r_obs, d_min, margin, tol)
    h_joints = joint_limit_barrier(q, q_min, q_max, margin);
    h_obs = obstacle_barrier(q, l1, l2, c, r_obs, d_min);
    safe = all(h_joints >= -tol) && (h_obs >= -tol);
end
