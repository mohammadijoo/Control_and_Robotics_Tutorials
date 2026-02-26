% Chapter12_Lesson1.m
% Autonomous Mobile Robots (Control Engineering)
% Chapter 12 — SLAM II: Graph-Based SLAM
% Lesson 1 — Pose Graphs and Factor Graphs
%
% A minimal 2D pose-graph optimizer (SE(2)) in MATLAB using Gauss–Newton.
% Also includes a small programmatic Simulink stub that generates a noisy
% odometry chain (optional) to create sequential constraints.

function Chapter12_Lesson1()
    rng(4);

    % --------
    % Ground truth trajectory (tiny loop)
    % --------
    gt = [...
        0 0 0;
        1 0 0;
        2 0 0;
        2 1 pi/2;
        2 2 pi/2;
        1 2 pi;
        0 2 pi;
        0 1 -pi/2];
    N = size(gt,1);

    % --------
    % Build noisy relative measurements z_ij
    % --------
    sigma_xy = 0.02;
    sigma_th = deg2rad(1.0);
    Omega = diag([1/sigma_xy^2, 1/sigma_xy^2, 1/sigma_th^2]);

    edges = {};
    for i=1:N-1
        z = between(gt(i,:), gt(i+1,:));
        z = z + [sigma_xy*randn, sigma_xy*randn, sigma_th*randn];
        z(3) = wrapAngle(z(3));
        edges{end+1} = struct('i', i, 'j', i+1, 'z', z, 'Omega', Omega);
    end
    % loop closure 8 -> 1
    zlc = between(gt(8,:), gt(1,:));
    zlc = zlc + [sigma_xy*randn, sigma_xy*randn, sigma_th*randn];
    zlc(3) = wrapAngle(zlc(3));
    edges{end+1} = struct('i', 8, 'j', 1, 'z', zlc, 'Omega', Omega);

    % --------
    % Initial guess
    % --------
    x0 = gt;
    x0(:,1:2) = x0(:,1:2) + 0.15*randn(N,2);
    x0(:,3) = arrayfun(@(a) wrapAngle(a), x0(:,3) + deg2rad(5)*randn(N,1));

    % --------
    % Prior on node 1 to fix gauge
    % --------
    Omega0 = diag([1e6, 1e6, 1e6]);
    priors = { struct('i', 1, 'mu', gt(1,:), 'Omega', Omega0) };

    fprintf('Optimizing...\n');
    est = gaussNewton(x0, edges, priors, 15);

    disp(' i | gt(x,y,th)               | est(x,y,th)');
    for i=1:N
        fprintf('%2d | [% .3f % .3f % .3f] | [% .3f % .3f % .3f]\n', ...
            i-1, gt(i,1), gt(i,2), gt(i,3), est(i,1), est(i,2), est(i,3));
    end

    % Optional: build a simple Simulink model that generates a noisy odometry chain
    % (This is illustrative; it is not required for optimization above.)
    buildSimulinkOdometryStub();
end

% --------------------------
% Core math helpers (SE(2))
% --------------------------
function a = wrapAngle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function R = rot(th)
    c = cos(th); s = sin(th);
    R = [c -s; s c];
end

function z = between(xi, xj)
    ti = xi(1:2)'; tj = xj(1:2)';
    thi = xi(3); thj = xj(3);
    dt = tj - ti;
    dxy = rot(thi)' * dt;
    dth = wrapAngle(thj - thi);
    z = [dxy' dth];
end

function xnew = boxplus(x, d)
    % d is in local frame: [dx dy dth]
    th = x(3);
    t = x(1:2)' + rot(th) * d(1:2)';
    xnew = [t' wrapAngle(th + d(3))];
end

function [r, Ji, Jj] = residualAndJacobians(xi, xj, z)
    % residual r = z - between(xi,xj)
    ti = xi(1:2)'; tj = xj(1:2)'; thi = xi(3);
    dt = tj - ti;
    RiT = rot(thi)';

    zhat = between(xi, xj);
    r = z - zhat; r(3) = wrapAngle(r(3));

    J = [0 -1; 1 0];

    Ji = zeros(3,3);
    Jj = zeros(3,3);

    Ji(1:2,1:2) = RiT;
    dri_dth = RiT * (J * dt);
    Ji(1:2,3) = dri_dth;
    Ji(3,3) = 1;

    Jj(1:2,1:2) = -RiT;
    Jj(3,3) = -1;
end

function est = gaussNewton(x0, edges, priors, iters)
    est = x0;
    N = size(x0,1);

    for k=1:iters
        [H, g] = buildNormalEquations(est, edges, priors);
        H = H + 1e-8*eye(3*N);

        dx = -H \ g;

        maxStep = 0;
        for i=1:N
            d = dx(3*i-2:3*i)';
            est(i,:) = boxplus(est(i,:), d);
            maxStep = max(maxStep, norm(d));
        end
        fprintf('iter %02d: max|delta| = %.3e\n', k-1, maxStep);
        if maxStep &lt; 1e-9, break; end
    end
end

function [H, g] = buildNormalEquations(poses, edges, priors)
    N = size(poses,1);
    H = zeros(3*N, 3*N);
    g = zeros(3*N, 1);

    sl = @(i) (3*i-2):(3*i); % i is 1-based

    % Relative edges
    for eidx=1:numel(edges)
        e = edges{eidx};
        i = e.i; j = e.j;
        [r, Ji, Jj] = residualAndJacobians(poses(i,:), poses(j,:), e.z);

        Omega = e.Omega;

        Hi  = Ji' * Omega * Ji;
        Hj  = Jj' * Omega * Jj;
        Hij = Ji' * Omega * Jj;

        gi = Ji' * Omega * r(:);
        gj = Jj' * Omega * r(:);

        si = sl(i); sj = sl(j);
        H(si,si) = H(si,si) + Hi;
        H(sj,sj) = H(sj,sj) + Hj;
        H(si,sj) = H(si,sj) + Hij;
        H(sj,si) = H(sj,si) + Hij';

        g(si) = g(si) + gi;
        g(sj) = g(sj) + gj;
    end

    % Priors
    for pidx=1:numel(priors)
        p = priors{pidx};
        i = p.i;
        mu = p.mu(:);
        xi = poses(i,:)'; xi(3) = wrapAngle(xi(3));
        r = mu - xi; r(3) = wrapAngle(r(3));
        J = -eye(3);
        Omega = p.Omega;

        si = sl(i);
        H(si,si) = H(si,si) + J' * Omega * J;
        g(si) = g(si) + J' * Omega * r;
    end
end

% --------------------------
% Simulink stub (optional)
% --------------------------
function buildSimulinkOdometryStub()
    % This stub constructs a simple Simulink model programmatically:
    % inputs v,w, integrate to x,y,theta using Euler.
    % It illustrates how a simulation can produce sequential constraints z_{k,k+1}.
    %
    % If Simulink is not installed/licensed, this function safely exits.

    if ~license('test','Simulink')
        fprintf('[Simulink] Not available. Skipping model creation.\n');
        return;
    end

    model = 'Chapter12_Lesson1_OdomStub';
    if bdIsLoaded(model), close_system(model, 0); end
    if exist([model '.slx'],'file'), delete([model '.slx']); end

    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Constant', [model '/v'], 'Value', '1.0', 'Position',[30 40 80 70]);
    add_block('simulink/Sources/Constant', [model '/w'], 'Value', '0.1', 'Position',[30 120 80 150]);
    add_block('simulink/Sources/Constant', [model '/dt'], 'Value', '0.05', 'Position',[30 200 80 230]);

    % MATLAB Function block for state update
    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/SE2 Integrator'], 'Position',[160 60 340 210]);
    set_param([model '/SE2 Integrator'], 'Script', sprintf([ ...
        'function [x,y,th]=f(v,w,dt)\n' ...
        '%% Simple Euler integrator for (x,y,th)\n' ...
        'persistent X Y TH;\n' ...
        'if isempty(X), X=0; Y=0; TH=0; end\n' ...
        'X = X + v*cos(TH)*dt;\n' ...
        'Y = Y + v*sin(TH)*dt;\n' ...
        'TH = TH + w*dt;\n' ...
        'x=X; y=Y; th=TH;\n' ...
        'end\n']));

    add_block('simulink/Sinks/To Workspace', [model '/x_out'], 'VariableName','x_out','Position',[410 60 470 90]);
    add_block('simulink/Sinks/To Workspace', [model '/y_out'], 'VariableName','y_out','Position',[410 110 470 140]);
    add_block('simulink/Sinks/To Workspace', [model '/th_out'], 'VariableName','th_out','Position',[410 160 470 190]);

    add_line(model, 'v/1', 'SE2 Integrator/1');
    add_line(model, 'w/1', 'SE2 Integrator/2');
    add_line(model, 'dt/1', 'SE2 Integrator/3');

    add_line(model, 'SE2 Integrator/1', 'x_out/1');
    add_line(model, 'SE2 Integrator/2', 'y_out/1');
    add_line(model, 'SE2 Integrator/3', 'th_out/1');

    save_system(model);
    fprintf('[Simulink] Created model: %s.slx\n', model);
end
