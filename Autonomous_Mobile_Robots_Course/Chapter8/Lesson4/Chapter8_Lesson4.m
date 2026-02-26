% Chapter8_Lesson4.m
% Particle-Filter Localization — Degeneracy + Kidnapped Robot Recovery (2D)
%
% Run:
%   Chapter8_Lesson4
%
% Notes:
% - This script simulates a 2D robot and range-bearing measurements to known landmarks,
%   runs MCL, monitors degeneracy via N_eff, and recovers from a "kidnapped robot"
%   using adaptive random particle injection.
% - At the end, it writes a CSV file "Chapter8_Lesson4_results.csv".
%
% Optional (Simulink):
% - Call buildSimulinkSkeleton() to auto-create a basic Simulink model with a
%   MATLAB Function block hosting the PF update (lightweight scaffold).

function Chapter8_Lesson4()
    rng(4);

    % Bounds and landmarks
    xmin = 0; xmax = 10; ymin = 0; ymax = 10;
    landmarks = [2 2; 8 2; 8 8; 2 8];

    % Simulation parameters
    dt = 0.1;
    T = 300;
    kidnapped_t = 170;

    % Noise
    sigma_v = 0.05; sigma_w = 0.03;
    sigma_r = 0.15; sigma_b = 0.07;

    % PF settings
    N = 800;
    Neff_ratio = 0.5;
    rough_k = 0.15;

    % Kidnapped recovery
    eps_min = 0.01;
    eps_max = 0.30;
    ll_thresh = -12.0;

    % True state
    x_true = [1.0; 1.0; 0.0];

    % Initialize particles globally
    P = zeros(N,3);
    P(:,1) = xmin + (xmax - xmin) * rand(N,1);
    P(:,2) = ymin + (ymax - ymin) * rand(N,1);
    P(:,3) = -pi + 2*pi*rand(N,1);
    w = ones(N,1)/N;

    true_hist = zeros(T,3);
    est_hist  = zeros(T,3);
    Neff_hist = zeros(T,1);
    eps_hist  = zeros(T,1);

    for t = 1:T
        [v_cmd, w_cmd] = controlProfile(t);

        % True motion (unicycle + noise)
        x_true = motionStep(x_true, v_cmd, w_cmd, dt, sigma_v, sigma_w);

        % Kidnapped event
        if t == kidnapped_t
            x_true = [xmin + (xmax - xmin)*rand();
                      ymin + (ymax - ymin)*rand();
                      -pi + 2*pi*rand()];
        end

        % Measurement from true pose
        z = measurementModel(x_true, landmarks);
        z(:,1) = z(:,1) + sigma_r * randn(size(z,1),1);
        z(:,2) = wrapAngle(z(:,2) + sigma_b * randn(size(z,1),1));

        % Predict particles
        for i = 1:N
            P(i,:) = motionStep(P(i,:).', v_cmd, w_cmd, dt, sigma_v, sigma_w).';
        end

        % Update weights in log domain
        ll = zeros(N,1);
        for i = 1:N
            ll(i) = logSensorLikelihood(P(i,:).', z, landmarks, sigma_r, sigma_b);
        end
        ll = ll - max(ll);
        w = exp(ll);
        w = w / sum(w);

        % Weighted mean log-likelihood for mismatch detection
        ll_w = zeros(N,1);
        for i = 1:N
            ll_w(i) = logSensorLikelihood(P(i,:).', z, landmarks, sigma_r, sigma_b);
        end
        lbar = sum(w .* ll_w);

        if lbar < ll_thresh
            eps = eps_max;
        else
            eps = eps_min;
        end

        % Degeneracy
        Neff = 1 / sum(w.^2);

        if Neff < Neff_ratio * N
            [P, w] = systematicResample(P, w);

            % Roughen
            P = roughen(P, rough_k, [xmin xmax ymin ymax]);

            % Inject random particles
            m = round(eps*N);
            if m > 0
                P(1:m,1) = xmin + (xmax - xmin) * rand(m,1);
                P(1:m,2) = ymin + (ymax - ymin) * rand(m,1);
                P(1:m,3) = -pi + 2*pi*rand(m,1);
            end

            % Re-weight after injection
            ll2 = zeros(N,1);
            for i = 1:N
                ll2(i) = logSensorLikelihood(P(i,:).', z, landmarks, sigma_r, sigma_b);
            end
            ll2 = ll2 - max(ll2);
            w = exp(ll2);
            w = w / sum(w);
        end

        x_est = estimatePose(P, w);

        true_hist(t,:) = x_true.';
        est_hist(t,:)  = x_est.';
        Neff_hist(t) = Neff;
        eps_hist(t)  = eps;
    end

    % Save CSV
    fid = fopen('Chapter8_Lesson4_results.csv','w');
    fprintf(fid, 't,true_x,true_y,true_th,est_x,est_y,est_th,Neff,eps\n');
    for t = 1:T
        fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            t-1, true_hist(t,1), true_hist(t,2), true_hist(t,3), ...
            est_hist(t,1), est_hist(t,2), est_hist(t,3), ...
            Neff_hist(t), eps_hist(t));
    end
    fclose(fid);
    disp('Saved: Chapter8_Lesson4_results.csv');

    % Plot
    figure; plot(true_hist(:,1), true_hist(:,2), 'LineWidth', 1.5); hold on;
    plot(est_hist(:,1), est_hist(:,2), 'LineWidth', 1.5);
    scatter(landmarks(:,1), landmarks(:,2), 60, 'x', 'LineWidth', 2);
    axis equal; grid on; legend('true','estimate','landmarks');
    title(sprintf('MCL trajectory (kidnapped at t=%d)', kidnapped_t));

    figure; plot(Neff_hist, 'LineWidth', 1.5); hold on;
    yline(Neff_ratio * N, '--');
    grid on; title('Effective sample size N_{eff}');

    figure; plot(eps_hist, 'LineWidth', 1.5);
    grid on; title('Injection fraction eps');

    % Uncomment to generate a Simulink scaffold:
    % buildSimulinkSkeleton();
end

function [v, w] = controlProfile(t)
    if t < 70
        v = 0.7; w = 0.0;
    elseif t < 90
        v = 0.7; w = 0.9;
    elseif t < 160
        v = 0.7; w = 0.0;
    elseif t < 180
        v = 0.7; w = 0.9;
    elseif t < 250
        v = 0.7; w = 0.0;
    else
        v = 0.7; w = 0.9;
    end
end

function x2 = motionStep(x, v, w, dt, sigma_v, sigma_w)
    v_n = v + sigma_v * randn();
    w_n = w + sigma_w * randn();
    if abs(w_n) < 1e-9
        x2 = [x(1) + v_n*dt*cos(x(3));
              x(2) + v_n*dt*sin(x(3));
              x(3)];
    else
        x2 = [x(1) + (v_n/w_n)*(sin(x(3)+w_n*dt) - sin(x(3)));
              x(2) - (v_n/w_n)*(cos(x(3)+w_n*dt) - cos(x(3)));
              x(3) + w_n*dt];
    end
    x2(3) = wrapAngle(x2(3));
end

function z = measurementModel(x, landmarks)
    dx = landmarks(:,1) - x(1);
    dy = landmarks(:,2) - x(2);
    r = sqrt(dx.^2 + dy.^2);
    b = atan2(dy, dx) - x(3);
    b = wrapAngle(b);
    z = [r, b];
end

function ll = logSensorLikelihood(xp, z, landmarks, sigma_r, sigma_b)
    zh = measurementModel(xp, landmarks);
    dr = z(:,1) - zh(:,1);
    db = wrapAngle(z(:,2) - zh(:,2));
    cr = -log(sqrt(2*pi)*sigma_r);
    cb = -log(sqrt(2*pi)*sigma_b);
    ll = sum(-0.5*(dr./sigma_r).^2 + cr) + sum(-0.5*(db./sigma_b).^2 + cb);
end

function a = wrapAngle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function [P2, w2] = systematicResample(P, w)
    N = size(P,1);
    cdf = cumsum(w);
    cdf(end) = 1.0;
    u0 = rand()/N;
    idx = zeros(N,1);
    i = 1;
    for m = 1:N
        u = u0 + (m-1)/N;
        while u > cdf(i)
            i = i + 1;
        end
        idx(m) = i;
    end
    P2 = P(idx,:);
    w2 = ones(N,1)/N;
end

function P2 = roughen(P, k, bounds)
    xmin = bounds(1); xmax = bounds(2);
    ymin = bounds(3); ymax = bounds(4);
    N = size(P,1);
    d = 3;
    sx  = k*(xmax-xmin)*N^(-1/d);
    sy  = k*(ymax-ymin)*N^(-1/d);
    sth = k*(2*pi)*N^(-1/d);
    P2 = P;
    P2(:,1) = P(:,1) + sx*randn(N,1);
    P2(:,2) = P(:,2) + sy*randn(N,1);
    P2(:,3) = wrapAngle(P(:,3) + sth*randn(N,1));
end

function xhat = estimatePose(P, w)
    x = sum(w .* P(:,1));
    y = sum(w .* P(:,2));
    c = sum(w .* cos(P(:,3)));
    s = sum(w .* sin(P(:,3)));
    th = atan2(s, c);
    xhat = [x; y; th];
end

function buildSimulinkSkeleton()
    model = 'Chapter8_Lesson4_MCL_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model); open_system(model);

    add_block('simulink/Sources/Constant', [model '/u_vw'], 'Value', '[0.7; 0.0]');
    add_block('simulink/Sources/Constant', [model '/z_meas'], 'Value', 'zeros(4,2)');

    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/MCL_Update']);
    set_param([model '/MCL_Update'], 'Script', sprintf([ ...
        'function xhat = f(u, z)\n' ...
        '%% u = [v; w], z = measurements\n' ...
        '%% This block is a scaffold. Paste your PF predict/update/resample here.\n' ...
        'xhat = zeros(3,1);\n' ...
        'end\n' ...
    ]));

    add_block('simulink/Sinks/Display', [model '/xhat_display']);

    add_line(model, 'u_vw/1', 'MCL_Update/1');
    add_line(model, 'z_meas/1', 'MCL_Update/2');
    add_line(model, 'MCL_Update/1', 'xhat_display/1');

    set_param(model, 'StopTime', '10');
    save_system(model);
    disp(['Created Simulink scaffold: ' model]);
end
