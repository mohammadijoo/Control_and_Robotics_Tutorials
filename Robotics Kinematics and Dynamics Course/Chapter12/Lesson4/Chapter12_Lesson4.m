function tau = newtonEulerSerial(R, p, r_c, m, I, q, qd, qdd, g0, jointType)
% R, p, r_c, I : cell arrays of length n
% m           : [n x 1] masses
% jointType   : char vector 'R' or 'P'
% q, qd, qdd  : [n x 1]
% g0          : [3 x 1] gravity in base frame

n = numel(m);
z = [0; 0; 1];

omega   = zeros(3, n+1);
omegad  = zeros(3, n+1);
a       = zeros(3, n+1);
a_c     = zeros(3, n);

% base
a(:,1) = -g0;

% forward recursion
for i = 1:n
    Ri = R{i};
    pi = p{i};
    jt = jointType(i);
    if jt == 'R'
        omega(:,i+1) = Ri * omega(:,i) + z * qd(i);
        omegad(:,i+1) = Ri * omegad(:,i) + z * qdd(i) ...
            + cross(omega(:,i+1), z * qd(i));
        a(:,i+1) = Ri * ( a(:,i) ...
            + cross(omegad(:,i), pi) ...
            + cross(omega(:,i), cross(omega(:,i), pi)) );
    else
        omega(:,i+1) = Ri * omega(:,i);
        omegad(:,i+1) = Ri * omegad(:,i);
        a(:,i+1) = Ri * ( a(:,i) ...
            + cross(omegad(:,i), pi) ...
            + cross(omega(:,i), cross(omega(:,i), pi)) ) ...
            + 2 * cross(omega(:,i+1), z * qd(i)) + z * qdd(i);
    end
    rc = r_c{i};
    a_c(:,i) = a(:,i+1) + cross(omegad(:,i+1), rc) ...
        + cross(omega(:,i+1), cross(omega(:,i+1), rc));
end

% backward recursion
f = zeros(3, n+2);
nvec = zeros(3, n+2);
tau = zeros(n,1);

for i = n:-1:1
    Ri_next = eye(3);
    pi_next = [0; 0; 0];
    if i < n
        Ri_next = R{i+1}';
        pi_next = p{i+1};
    end
    mi = m(i);
    Ii = I{i};
    rc = r_c{i};

    f(:,i+1) = mi * a_c(:,i) + Ri_next * f(:,i+2);
    nvec(:,i+1) = Ii * omegad(:,i+1) + cross(omega(:,i+1), Ii * omega(:,i+1)) ...
        + cross(rc, mi * a_c(:,i)) ...
        + Ri_next * nvec(:,i+2) ...
        + cross(pi_next + rc, Ri_next * f(:,i+2));

    if jointType(i) == 'R'
        tau(i) = dot(nvec(:,i+1), z);
    else
        tau(i) = dot(f(:,i+1), z);
    end
end
end
      
