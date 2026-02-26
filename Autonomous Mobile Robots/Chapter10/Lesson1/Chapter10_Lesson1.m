% Chapter10_Lesson1.m
% Point-Cloud / Scan Registration Goals — MATLAB + Simulink script
%
% This file demonstrates the registration *goal* for 2D scans:
% estimate a rigid transform (R,t) aligning paired points.
%
% Part A: Closed-form Kabsch/Procrustes in 2D.
% Part B: Programmatically build a tiny Simulink model that calls a MATLAB Function
%         block to estimate the same transform for a fixed dataset.

function Chapter10_Lesson1()
    rng(7);

    % Synthetic paired points
    N = 200;
    P = [ -5 + 10*rand(N,1), -3 + 6*rand(N,1) ];

    theta_gt = deg2rad(12.0);
    t_gt = [1.2; -0.7];
    R_gt = [cos(theta_gt), -sin(theta_gt); sin(theta_gt), cos(theta_gt)];

    sigma = 0.02;
    Q = (R_gt * P')' + repmat(t_gt', N, 1) + sigma*randn(N,2);

    [R_hat, t_hat] = kabsch2d(P, Q);

    theta_hat = atan2(R_hat(2,1), R_hat(1,1));

    fprintf('theta_gt(deg)=%.6f, t_gt=[%.4f %.4f]\\n', rad2deg(theta_gt), t_gt(1), t_gt(2));
    fprintf('theta_hat(deg)=%.6f, t_hat=[%.4f %.4f]\\n', rad2deg(theta_hat), t_hat(1), t_hat(2));

    Q_hat = (R_hat * P')' + repmat(t_hat', N, 1);
    err = sqrt(mean(sum((Q_hat - Q).^2, 2)));
    fprintf('RMSE(Q_hat,Q)=%.6f\\n', err);

    % Optional: build a minimal Simulink model
    build_simulink_model(P, Q);
end

function [R, t] = kabsch2d(P, Q)
    % P,Q: N x 2
    pbar = mean(P, 1);
    qbar = mean(Q, 1);

    X = P - pbar;
    Y = Q - qbar;

    H = X' * Y;
    [U,~,V] = svd(H);
    R = V * U';

    if det(R) < 0
        V(:,2) = -V(:,2);
        R = V * U';
    end

    t = qbar' - R * pbar';
end

function build_simulink_model(P, Q)
    % Creates a Simulink model "Chapter10_Lesson1_Simulink" in the current folder.
    % Requires Simulink.
    mdl = 'Chapter10_Lesson1_Simulink';

    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end

    new_system(mdl);
    open_system(mdl);

    % Save P and Q to base workspace for Constant blocks
    assignin('base', 'P_data', P);
    assignin('base', 'Q_data', Q);

    % Add Constant blocks
    add_block('simulink/Sources/Constant', [mdl '/P'], 'Value', 'P_data', 'Position', [30 50 120 80]);
    add_block('simulink/Sources/Constant', [mdl '/Q'], 'Value', 'Q_data', 'Position', [30 120 120 150]);

    % MATLAB Function block to estimate transform
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/EstimateSE2'], 'Position', [200 70 360 150]);

    % Add Outports
    add_block('simulink/Sinks/Out1', [mdl '/R'], 'Position', [430 80 460 100]);
    add_block('simulink/Sinks/Out1', [mdl '/t'], 'Position', [430 130 460 150]);

    % Wire
    add_line(mdl, 'P/1', 'EstimateSE2/1');
    add_line(mdl, 'Q/1', 'EstimateSE2/2');
    add_line(mdl, 'EstimateSE2/1', 'R/1');
    add_line(mdl, 'EstimateSE2/2', 't/1');

    % Set MATLAB Function code
    code = [
        "function [R,t] = f(P,Q)" newline ...
        "%#codegen" newline ...
        "pbar = mean(P,1); qbar = mean(Q,1);" newline ...
        "X = P - pbar; Y = Q - qbar;" newline ...
        "H = X' * Y;" newline ...
        "[U,~,V] = svd(H);" newline ...
        "R = V * U';" newline ...
        "if det(R) < 0" newline ...
        "  V(:,2) = -V(:,2);" newline ...
        "  R = V * U';" newline ...
        "end" newline ...
        "t = qbar' - R * pbar';" newline ...
        "end"
    ];
    set_param([mdl '/EstimateSE2'], 'Script', code);

    save_system(mdl);
    fprintf('Simulink model created: %s.slx\\n', mdl);
end
