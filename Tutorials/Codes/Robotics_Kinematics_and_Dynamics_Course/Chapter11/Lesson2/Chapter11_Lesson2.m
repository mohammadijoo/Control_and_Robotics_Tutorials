syms q1 q2 dq1 dq2 a1 a2 a3 real
q  = [q1; q2];
dq = [dq1; dq2];

% Inertia matrix
M = [a1 + 2*a2*cos(q2), a3 + a2*cos(q2);
     a3 + a2*cos(q2),   a3];

n = 2;
c = sym(zeros(n, n, n));

for i = 1:n
  for j = 1:n
    for k = 1:n
      c(i,j,k) = 1/2 * ( diff(M(i,j), q(k)) ...
                       + diff(M(i,k), q(j)) ...
                       - diff(M(j,k), q(i)) );
    end
  end
end

C = sym(zeros(n, n));
for i = 1:n
  for j = 1:n
    C(i,j) = c(i,j,1)*dq1 + c(i,j,2)*dq2;
  end
end

C = simplify(C)

% Export to a numeric MATLAB function for Simulink
matlabFunction(C, 'File', 'Coriolis2R', ...
    'Vars', [q1, q2, dq1, dq2, a1, a2, a3]);
      
