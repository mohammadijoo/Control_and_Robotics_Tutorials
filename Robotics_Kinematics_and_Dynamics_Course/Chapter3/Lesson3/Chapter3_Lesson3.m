function q = quat_from_axis_angle(axis, theta)
    axis = axis(:) / norm(axis);
    half = 0.5 * theta;
    s = sin(half);
    q = [cos(half); axis * s];  % [w; x; y; z]
end

function q = quat_normalize(q)
    q = q(:);
    q = q / norm(q);
end

function q = quat_multiply(q1, q2)
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    q = [w; x; y; z];
end

function v_rot = quat_rotate(q, v)
    q = quat_normalize(q);
    vq = [0; v(:)];
    q_conj = [q(1); -q(2:4)];
    res = quat_multiply(quat_multiply(q, vq), q_conj);
    v_rot = res(2:4);
end

function q = quat_slerp(q0, q1, t)
    q0 = quat_normalize(q0);
    q1 = quat_normalize(q1);
    dot = dot(q0, q1);

    if dot < 0
        q1 = -q1;
        dot = -dot;
    end

    dot = max(-1.0, min(1.0, dot));
    theta = acos(dot);

    if theta < 1e-6
        q = quat_normalize((1 - t)*q0 + t*q1);
        return;
    end

    sin_theta = sin(theta);
    w0 = sin((1 - t)*theta) / sin_theta;
    w1 = sin(t*theta) / sin_theta;
    q = quat_normalize(w0*q0 + w1*q1);
end

% Example script
q0 = quat_from_axis_angle([0;0;1], 0);
q1 = quat_from_axis_angle([0;0;1], pi/2);
t = 0.5;
qm = quat_slerp(q0, q1, t)
v = [1; 0; 0];
v_rot = quat_rotate(qm, v)
      
