
function dx = twolink_ctc(t, x)
    % x = [q1; q2; q1d; q2d]
    q  = x(1:2);
    qd = x(3:4);

    % True parameters
    p.l1 = 1.0; p.l2 = 1.0;
    p.m1 = 3.0; p.m2 = 2.0;
    p.I1 = 0.2; p.I2 = 0.1;
    p.g  = 9.81;

    % Nominal parameters (controller)
    pn = p;
    pn.m1 = 2.5; pn.m2 = 1.5;

    % Desired trajectory
    w = 0.5;
    qd_d  = [0.5*sin(w*t); 0.3*sin(w*t)];
    qd1_d = [0.5*w*cos(w*t); 0.3*w*cos(w*t)];
    qd2_d = [-0.5*w^2*sin(w*t); -0.3*w^2*sin(w*t)];

    e  = q  - qd_d;
    ed = qd - qd1_d;

    Kp = diag([25, 16]);
    Kd = diag([10, 8]);

    Mn = Mmat(q, pn);
    Cn = Cmat(q, qd, pn);
    gn = gvec(q, pn);

    v   = qd2_d - Kd*ed - Kp*e;
    tau = Mn*v + Cn*qd + gn;

    Mr = Mmat(q, p);
    Cr = Cmat(q, qd, p);
    gr = gvec(q, p);

    qdd = Mr \ (tau - Cr*qd - gr);
    dx = [qd; qdd];
end

function M = Mmat(q, p)
    q2 = q(2);
    c2 = cos(q2);
    m11 = p.I1 + p.I2 + p.m1*(p.l1^2)/4 + p.m2*(p.l1^2 + (p.l2^2)/4 + p.l1*p.l2*c2);
    m12 = p.I2 + p.m2*((p.l2^2)/4 + 0.5*p.l1*p.l2*c2);
    m22 = p.I2 + p.m2*(p.l2^2)/4;
    M = [m11 m12; m12 m22];
end

function C = Cmat(q, qd, p)
    q2  = q(2);
    q1d = qd(1); q2d = qd(2);
    s2 = sin(q2);
    c11 = -p.m2*p.l1*p.l2*s2*q2d;
    c12 = -p.m2*p.l1*p.l2*s2*(q1d + q2d);
    c21 =  p.m2*p.l1*p.l2*s2*q1d;
    c22 = 0;
    C = [c11 c12; c21 c22];
end

function g = gvec(q, p)
    q1 = q(1); q2 = q(2);
    g1 = (p.m1*p.l1/2 + p.m2*p.l1)*p.g*cos(q1) + p.m2*p.l2/2*p.g*cos(q1 + q2);
    g2 = p.m2*p.l2/2*p.g*cos(q1 + q2);
    g = [g1; g2];
end
