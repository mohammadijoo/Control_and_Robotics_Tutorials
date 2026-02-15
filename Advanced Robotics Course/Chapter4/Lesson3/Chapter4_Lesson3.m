function traj = trajopt_sco_matlab(robot, qStart, qGoal, obstacles)
% TRAJOPT_SCO_MATLAB
%   robot     : rigidBodyTree from Robotics System Toolbox
%   qStart    : 1xn start configuration
%   qGoal     : 1xn goal configuration
%   obstacles : struct array with fields center (1x3), radius

N = 20;
lambdaSmooth = 1.0;
trustRadius  = 0.3;
dMin         = 0.05;
n            = numel(qStart);

% Initial trajectory (linear interpolation)
traj = zeros(N+1, n);
for k = 0:N
    s = k / N;
    traj(k+1,:) = (1 - s) * qStart + s * qGoal;
end

for it = 1:10
    % Decision vector x stacks all q_k
    nvar = (N+1) * n;
    H = zeros(nvar, nvar);
    f = zeros(nvar, 1);

    % Smoothness term
    for k = 2:N
        for j = 1:n
            idxPrev = (k-2)*n + j;
            idxCurr = (k-1)*n + j;
            idxNext = k*n + j;

            H(idxPrev, idxPrev) = H(idxPrev, idxPrev) + lambdaSmooth;
            H(idxCurr, idxCurr) = H(idxCurr, idxCurr) + 4*lambdaSmooth;
            H(idxNext, idxNext) = H(idxNext, idxNext) + lambdaSmooth;

            H(idxPrev, idxCurr) = H(idxPrev, idxCurr) - 2*lambdaSmooth;
            H(idxCurr, idxPrev) = H(idxCurr, idxPrev) - 2*lambdaSmooth;

            H(idxCurr, idxNext) = H(idxCurr, idxNext) - 2*lambdaSmooth;
            H(idxNext, idxCurr) = H(idxNext, idxCurr) - 2*lambdaSmooth;

            H(idxPrev, idxNext) = H(idxPrev, idxNext) + lambdaSmooth;
            H(idxNext, idxPrev) = H(idxNext, idxPrev) + lambdaSmooth;
        end
    end

    % Goal term ||q_N - qGoal||^2
    for j = 1:n
        idx = N*n + j;
        H(idx, idx) = H(idx, idx) + 1;
        f(idx)      = f(idx) - 2*qGoal(j);
    end

    A = [];
    b = [];
    Aeq = [];
    beq = [];

    % Equality constraints for endpoints
    % q_0 = qStart, q_N = qGoal
    Aeq = zeros(2*n, nvar);
    beq = zeros(2*n, 1);
    % q_0
    for j = 1:n
        idx = j;
        Aeq(j, idx) = 1;
        beq(j)      = qStart(j);
    end
    % q_N
    for j = 1:n
        idx = N*n + j;
        Aeq(n+j, idx) = 1;
        beq(n+j)      = qGoal(j);
    end

    % Bounds (trust region)
    lb = zeros(nvar,1);
    ub = zeros(nvar,1);
    for k = 0:N
        for j = 1:n
            idx = k*n + j;
            qk  = traj(k+1,j);
            lb(idx) = qk - trustRadius;
            ub(idx) = qk + trustRadius;
        end
    end

    % Linearized collision constraints: A_coll * x >= b_coll
    % quadprog expects A*x <= b, so we multiply by -1.
    A_coll = [];
    b_coll = [];

    for k = 0:N
        qk = traj(k+1,:);
        % Compute collision points and normals using Robotics System Toolbox
        % (e.g., use checkCollision or custom signed-distance queries)
        for obs = obstacles
            % Simple point on end effector for illustration:
            [pose, J] = forwardKinematics(robot, qk); %#ok<NASGU>
            % Here, pose and J would be obtained via getTransform and geometricJacobian.
            % Suppose we have distance d and normal n such that
            % d(q) approx d(qk) + n * J * (q - qk)'
            % For brevity, we assume d and n are given:
            d = 0.1; % placeholder
            nvec = [1 0 0]; % placeholder

            a = nvec * J;       % row vector 1xn
            rhs = dMin - d + a*qk';

            row = zeros(1, nvar);
            for j = 1:n
                idx = k*n + j;
                row(idx) = a(j);
            end
            A_coll = [A_coll; -row];
            b_coll = [b_coll; -rhs];
        end
    end

    A = [A; A_coll];
    b = [b; b_coll];

    options = optimoptions('quadprog','Display','off');
    [xOpt,~,exitflag] = quadprog(H, f, A, b, Aeq, beq, lb, ub, [], options);
    if exitflag <= 0
        warning('quadprog failed at iteration %d', it);
        break;
    end

    traj_new = reshape(xOpt, [n, N+1])';
    if norm(traj_new - traj, 'fro') < 1e-3
        traj = traj_new;
        break;
    end
    traj = traj_new;
end
end
      
