syms q1 q2 dq1 dq2 real
syms g l1 l2 lc1 lc2 m1 m2 I1 I2 real

q  = [q1; q2];
dq = [dq1; dq2];

M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2));
M12 = I2 + m2*(lc2^2 + l1*lc2*cos(q2));
M22 = I2 + m2*lc2^2;

M = [M11, M12;
     M12, M22];

T = sym(0.5) * dq.' * M * dq;
V = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);

% Gravity vector
g_vec = jacobian(V, q).';

% Christoffel-based C(q, dq)
C = sym(zeros(2,2));
for i = 1:2
  for j = 1:2
    Cij = sym(0);
    for k = 1:2
      c_ijk = 0.5 * ( diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i)) );
      Cij = Cij + c_ijk * dq(k);
    end
    C(i,j) = simplify(Cij);
  end
end

% Generate numeric functions
matlabFunction(M, 'File', 'M_2R', ...
    'Vars', {q1, q2, l1, l2, lc1, lc2, m1, m2, I1, I2});
matlabFunction(C, 'File', 'C_2R', ...
    'Vars', {q1, q2, dq1, dq2, l1, l2, lc1, lc2, m1, m2, I1, I2});
matlabFunction(g_vec, 'File', 'g_2R', ...
    'Vars', {q1, q2, g, l1, l2, lc1, lc2, m1, m2});
      
