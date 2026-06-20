m = 1.0;
g = 9.81;
R = 1.0;
alpha = 10.0;
beta = 20.0;

h = 1e-3;
n_steps = 10000;

q = [R; 0.0];    % position [x;y]
v = [0.0; 0.5];  % velocity [vx;vy]

phi = @(q) q(1)^2 + q(2)^2 - R^2;
J   = @(q) [2*q(1), 2*q(2)];
Jdot = @(v) [2*v(1), 2*v(2)];

for k = 1:n_steps
    M = m * eye(2);
    g_vec = [0.0; m*g];

    Jq = J(q);
    Jd = Jdot(v);
    phi_q = phi(q);
    Jv = Jq * v;
    Jdv = Jd * v;

    % Build 3x3 block matrix
    A = [M, Jq.';
         Jq, 0];

    rhs = zeros(3,1);
    rhs(1:2) = -g_vec;
    rhs(3) = -Jdv - 2*alpha*Jv - beta^2*phi_q;

    sol = A \ rhs;
    vdot = sol(1:2);
    % lambda = sol(3);

    v = v + h * vdot;
    q = q + h * v;
end

fprintf('Final constraint value phi(q) = %.6e\n', phi(q));
      
