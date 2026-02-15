function planar2R_demo()
    % Parameters
    l1  = 1.0;  l2  = 1.0;
    lc1 = 0.5;  lc2 = 0.5;
    m1  = 1.0;  m2  = 1.0;
    I1  = 0.1;  I2  = 0.1;
    g   = 9.81;

    % Example state and input
    q   = [pi/4; pi/6];
    qd  = [0.2; -0.1];
    qdd = [0.5; 0.1];

    [Mq, Cq, gq] = planar2R_dynamics_matrices(q, qd, ...
                                              l1, l2, lc1, lc2, ...
                                              m1, m2, I1, I2, g);
    u = Mq * qdd + Cq * qd + gq;

    disp('Inertia matrix M(q):');
    disp(Mq);
    disp('Coriolis/centrifugal matrix C(q,qd):');
    disp(Cq);
    disp('Gravity vector g(q):');
    disp(gq);
    disp('Required joint inputs u:');
    disp(u);
end

function [Mq, Cq, gq] = planar2R_dynamics_matrices(q, qd, ...
                                                   l1, l2, lc1, lc2, ...
                                                   m1, m2, I1, I2, g)
    q1  = q(1);  q2  = q(2);
    q1d = qd(1); q2d = qd(2);

    c2 = cos(q2);
    s2 = sin(q2);

    % Inertia matrix
    M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2);
    M12 = I2 + m2*(lc2^2 + l1*lc2*c2);
    M22 = I2 + m2*lc2^2;
    Mq  = [M11, M12;
           M12, M22];

    % Coriolis/centrifugal
    h   = -m2*l1*lc2*s2;
    C11 = h*q2d;
    C12 = h*(q1d + q2d);
    C21 = -h*q1d;
    C22 = 0;
    Cq  = [C11, C12;
           C21, C22];

    % Gravity
    g1  = ((m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1+q2));
    g2  = m2*lc2*g*cos(q1+q2);
    gq  = [g1; g2];
end
      
