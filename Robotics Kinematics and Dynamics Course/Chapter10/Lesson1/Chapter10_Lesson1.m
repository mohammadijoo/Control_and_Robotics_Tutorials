function [T, V, M] = two_link_energy(q, qd, p)
% q, qd: 2x1 joint vectors
% p: struct with fields m1, m2, l1, l2, c1, c2, I1, I2, g

q1 = q(1);
q2 = q(2);

cos2 = cos(q2);

M11 = p.I1 + p.I2 ...
    + p.m1 * p.c1^2 ...
    + p.m2 * (p.l1^2 + p.c2^2 + 2 * p.l1 * p.c2 * cos2);
M12 = p.I2 + p.m2 * (p.c2^2 + p.l1 * p.c2 * cos2);
M22 = p.I2 + p.m2 * p.c2^2;

M = [M11, M12;
     M12, M22];

T = 0.5 * qd.' * M * qd;

V = p.m1 * p.g * p.c1 * sin(q1) ...
  + p.m2 * p.g * (p.l1 * sin(q1) + p.c2 * sin(q1 + q2));
end
      
