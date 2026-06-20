% Parameters
T  = 5.0;          % duration [s]
dt = 0.002;
t  = (0:dt:T).';
N  = numel(t);
nq = 6;            % 6-DOF arm
M  = 4;            % harmonics per joint

q0   = zeros(1, nq);
A    = 0.4 * ones(nq, M);
W    = linspace(1.0, 6.0, M);
PHI0 = linspace(0.0, pi/2, M);

q   = zeros(N, nq);
qd  = zeros(N, nq);
qdd = zeros(N, nq);

for i = 1:nq
    qi  = q0(i) * ones(N, 1);
    qdi = zeros(N, 1);
    qddi = zeros(N, 1);
    for m = 1:M
        w  = W(m);
        a  = A(i, m);
        ph = PHI0(m);
        s  = sin(w * t + ph);
        c  = cos(w * t + ph);
        qi  = qi  + a * s;
        qdi = qdi + a * w * c;
        qddi = qddi - a * w^2 * s;
    end
    q(:, i)   = qi;
    qd(:, i)  = qdi;
    qdd(:, i) = qddi;
end

% Export as Simulink timeseries for joint commands
excitation.q   = timeseries(q, t);
excitation.qd  = timeseries(qd, t);
excitation.qdd = timeseries(qdd, t);

% (In Simulink)
%  - Use "From Workspace" blocks to feed excitation.q to a joint trajectory
%    controller that tracks q(t).
%  - Measure joint torques tau_meas and joint positions.
%  - After simulation, build the regressor matrix Phi and evaluate
%    F = Phi' * Phi to check excitation quality.

% Example: compute information matrix from offline regressor function
Phi = [];
for k = 1:N
    qk   = q(k, :).';
    qdk  = qd(k, :).';
    qddk = qdd(k, :).';
    Yk = myRobotRegressor(qk, qdk, qddk);   % user-defined
    Phi = [Phi; Yk]; %#ok<AGROW>
end

F = Phi' * Phi;
lambda = eig(F);
lambda_min = min(lambda);
condF = max(lambda) / lambda_min;
fprintf('lambda_min = %.3e, cond(F) = %.3e\n', lambda_min, condF);
      
