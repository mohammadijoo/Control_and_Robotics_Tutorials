g = 9.81;

% Regressor row for over-parameterized pendulum
% tau = [qdd, 0, qdd, g*sin(q)] * [I0; m; m*c^2; m*c]
pendulumRegressorRow = @(q, qdd) [qdd, 0, qdd, g * sin(q)];

% Generate samples
N  = 50;
qs   = (rand(N,1) - 0.5) * 2*pi;
qdds = (rand(N,1) - 0.5) * 10;

W = zeros(N, 4);
for k = 1:N
    W(k,:) = pendulumRegressorRow(qs(k), qdds(k));
end

% SVD
[U, S, V] = svd(W, "econ");
singularVals = diag(S);
tol = 1e-8;
r = sum(singularVals > tol);

V1 = V(:,1:r);
V2 = V(:,r+1:end);

disp("Rank r = " + r);
disp("V1 (base parameter basis):");
disp(V1);
disp("V2 (null-space basis):");
disp(V2);

% Example full parameter vector
pi_full = [0.1; 2.0; 0.5; 1.0];
beta = V1.' * pi_full;
disp("Base parameters beta:");
disp(beta);
      
