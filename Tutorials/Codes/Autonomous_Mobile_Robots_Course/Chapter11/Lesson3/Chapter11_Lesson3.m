% Chapter11_Lesson3.m
% FastSLAM 1.0 (Rao–Blackwellized Particle Filter SLAM) — educational MATLAB script.
%
% Assumptions:
% - 2D pose [x;y;theta]
% - Known data association (each observation includes landmark id)
% - Range-bearing sensor; Gaussian noise
%
% This script includes:
% 1) Simple simulator
% 2) FastSLAM update loop with per-landmark EKF inside each particle
%
% Note: This is designed for clarity more than performance.

function Chapter11_Lesson3()
    rng(42);

    % Landmarks in world
    L = [2 2;
         8 1;
         6.5 7;
         1 8;
         9 9];
    M = size(L,1);

    % Settings
    T = 150; dt = 0.1; maxRange = 7.0;

    % Noise covariances
    R_u = diag([0.05^2, (2*pi/180)^2]);      % noise on [v,w]
    Q_z = diag([0.15^2, (3*pi/180)^2]);      % noise on [range,bearing]

    % Controls
    U = zeros(2,T);
    for t=1:T
        U(:,t) = [0.8; 0.25*sin(0.05*t)];
    end

    % Ground truth simulation + observations (cell array)
    xTrue = [0;0;0];
    xTrueHist = zeros(3,T);
    Z = cell(T,1);

    for t=1:T
        xTrue = noisyMotion(xTrue, U(:,t), dt, R_u);
        xTrueHist(:,t) = xTrue;

        obs = [];
        for i=1:M
            zhat = hLandmark(xTrue, L(i,:)');
            if zhat(1) <= maxRange
                z = zhat + mvnrnd([0 0], Q_z)';
                z(2) = wrapAngle(z(2));
                obs = [obs; i, z(1), z(2)]; %#ok<AGROW>
            end
        end
        Z{t} = obs;
    end

    % FastSLAM particles
    N = 200;
    particles = initParticles(N, M);

    xEstHist = zeros(3,T);

    for t=1:T
        particles = fastslamStep(particles, U(:,t), Z{t}, dt, R_u, Q_z);
        xEstHist(:,t) = estimatePose(particles);
    end

    fprintf('Final true pose: [%.3f %.3f %.3f]\n', xTrueHist(:,end));
    fprintf('Final est  pose: [%.3f %.3f %.3f]\n', xEstHist(:,end));

    figure; hold on; grid on; axis equal;
    plot(xTrueHist(1,:), xTrueHist(2,:), 'LineWidth', 1.5);
    plot(xEstHist(1,:), xEstHist(2,:), 'LineWidth', 1.5);
    scatter(L(:,1), L(:,2), 80, 'x', 'LineWidth', 2);
    legend('True', 'FastSLAM mean', 'Landmarks');
    title('FastSLAM 1.0 demo (known data association)');
end

% ------------------------- Core Functions -------------------------

function particles = initParticles(N, M)
    particles = struct();
    for i=1:N
        particles(i).x = [0;0;0];
        particles(i).w = 1.0/N;
        particles(i).lmSeen = false(M,1);
        particles(i).lmMu = zeros(2,M);
        particles(i).lmSigma = repmat(eye(2)*1e6, 1, 1, M);
    end
end

function particles = fastslamStep(particles, u, obs, dt, R_u, Q_z)
    N = numel(particles);

    % 1) motion sampling
    for i=1:N
        particles(i).x = noisyMotion(particles(i).x, u, dt, R_u);
    end

    % 2) landmark EKFs + weights
    for i=1:N
        x = particles(i).x;
        for k=1:size(obs,1)
            id = obs(k,1);
            z = obs(k,2:3)';

            if ~particles(i).lmSeen(id)
                m0 = invMeasurement(x, z);
                particles(i).lmMu(:,id) = m0;
                H = H_landmark(x, m0);
                J = inv(H);
                particles(i).lmSigma(:,:,id) = J * Q_z * J';
                particles(i).lmSeen(id) = true;
                continue;
            end

            mu = particles(i).lmMu(:,id);
            Sigma = particles(i).lmSigma(:,:,id);

            zhat = hLandmark(x, mu);
            v = [z(1)-zhat(1); wrapAngle(z(2)-zhat(2))];

            H = H_landmark(x, mu);
            S = H*Sigma*H' + Q_z;
            K = Sigma*H' / S;
            mu2 = mu + K*v;
            Sigma2 = (eye(2)-K*H)*Sigma;

            particles(i).lmMu(:,id) = mu2;
            particles(i).lmSigma(:,:,id) = Sigma2;

            particles(i).w = particles(i).w * gaussLike(v, S);
        end
    end

    % 3) normalize and resample if needed
    ws = [particles.w]';
    sw = sum(ws);
    if sw < 1e-30
        for i=1:N, particles(i).w = 1.0/N; end
    else
        for i=1:N, particles(i).w = particles(i).w / sw; end
    end

    ws = [particles.w]';
    ess = 1.0 / sum(ws.^2);
    if ess < 0.5*N
        particles = systematicResample(particles);
    end
end

function xhat = estimatePose(particles)
    ws = [particles.w]';
    ws = ws / max(sum(ws), 1e-12);

    X = reshape([particles.x], 3, [])';
    px = sum(ws .* X(:,1));
    py = sum(ws .* X(:,2));
    c = sum(ws .* cos(X(:,3)));
    s = sum(ws .* sin(X(:,3)));
    th = atan2(s, c);

    xhat = [px; py; th];
end

function parts = systematicResample(particles)
    N = numel(particles);
    ws = [particles.w]';
    ws = ws / max(sum(ws), 1e-12);
    cdf = cumsum(ws);

    u0 = rand() / N;
    idx = zeros(N,1);
    j = 1;
    for i=1:N
        u = u0 + (i-1)/N;
        while (j < N) && (u > cdf(j)), j = j + 1; end
        idx(i) = j;
    end

    parts = particles(idx);
    for i=1:N, parts(i).w = 1.0/N; end
end

% ---------------------- Models and Utilities ----------------------

function x2 = noisyMotion(x, u, dt, R_u)
    uNoisy = u + mvnrnd([0 0], R_u)';
    x2 = motionModel(x, uNoisy, dt);
end

function x2 = motionModel(x, u, dt)
    px = x(1); py = x(2); th = x(3);
    v = u(1); w = u(2);

    if abs(w) < 1e-9
        px2 = px + v*dt*cos(th);
        py2 = py + v*dt*sin(th);
        th2 = th;
    else
        px2 = px + (v/w)*(sin(th+w*dt)-sin(th));
        py2 = py - (v/w)*(cos(th+w*dt)-cos(th));
        th2 = th + w*dt;
    end
    x2 = [px2; py2; wrapAngle(th2)];
end

function z = hLandmark(x, m)
    px = x(1); py = x(2); th = x(3);
    dx = m(1) - px;
    dy = m(2) - py;
    q = dx^2 + dy^2;
    r = sqrt(max(q, 1e-12));
    b = wrapAngle(atan2(dy, dx) - th);
    z = [r; b];
end

function H = H_landmark(x, m)
    px = x(1); py = x(2);
    dx = m(1) - px;
    dy = m(2) - py;
    q = max(dx^2 + dy^2, 1e-12);
    r = sqrt(max(q, 1e-12));
    H = [dx/r, dy/r;
         -dy/q, dx/q];
end

function m = invMeasurement(x, z)
    px = x(1); py = x(2); th = x(3);
    r = z(1); b = z(2);
    ang = th + b;
    m = [px + r*cos(ang); py + r*sin(ang)];
end

function p = gaussLike(v, S)
    S = 0.5*(S+S');
    detS = max(det(S), 1e-18);
    invS = inv(S);
    e = v'*invS*v;
    p = (1/(2*pi*sqrt(detS))) * exp(-0.5*e);
end

function a = wrapAngle(a)
    while a <= -pi, a = a + 2*pi; end
    while a >  pi, a = a - 2*pi; end
end
