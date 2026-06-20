% Chapter7_Lesson1.m
% Chapter 7 - Lesson 1: EKF Localization Pipeline (mobile framing)
%
% Minimal EKF localization demo (2D) in MATLAB (from scratch).
% State: [x; y; theta] in world frame, Control: [v; omega] in body frame
% Measurement: range-bearing to known landmarks.

function Chapter7_Lesson1()
    rng(7);

    landmarks = [ 5 0;
                  5 5;
                  0 5;
                 -3 2 ];

    dt = 0.1;
    N  = 250;

    % True noise on control (simulation)
    sigma_v_true = 0.05;
    sigma_w_true = 0.03;

    % Assumed process noise in EKF (tuning)
    Q = diag([0.08^2, 0.05^2]);

    % Measurement noise (range, bearing)
    R = diag([0.15^2, (2*pi/180)^2]);

    % Initial truth and estimate
    x_true = [0; 0; 0];
    x_est  = [-0.2; 0.1; 0.05];
    P      = diag([0.5^2, 0.5^2, (10*pi/180)^2]);

    sumPos2 = 0;
    sumTh2  = 0;

    for k = 1:N
        v_cmd = 0.8 + 0.2*sin(0.04*(k-1));
        w_cmd = 0.25 + 0.05*cos(0.03*(k-1));
        u_cmd = [v_cmd; w_cmd];

        u_true = [v_cmd + sigma_v_true*randn();
                  w_cmd + sigma_w_true*randn()];

        % True propagation
        x_true = motion_model(x_true, u_true, dt);

        % EKF predict
        F = jacobian_F(x_est, u_cmd, dt);
        L = jacobian_L(x_est, dt);
        x_est = motion_model(x_est, u_cmd, dt);
        P = F*P*F' + L*Q*L';
        P = 0.5*(P + P');

        % Measurement (nearest landmark)
        idx = nearest_landmark(x_true, landmarks);
        lm  = landmarks(idx,:);

        z_true = meas_model(x_true, lm);
        z_meas = [z_true(1) + sqrt(R(1,1))*randn();
                  wrap_angle(z_true(2) + sqrt(R(2,2))*randn())];

        % EKF update
        H = jacobian_H(x_est, lm);
        zhat = meas_model(x_est, lm);

        y = z_meas - zhat;
        y(2) = wrap_angle(y(2));

        S = H*P*H' + R;
        K = P*H'/S;

        x_est = x_est + K*y;
        x_est(3) = wrap_angle(x_est(3));

        I = eye(3);
        P = (I - K*H)*P*(I - K*H)' + K*R*K';
        P = 0.5*(P + P');

        ex = x_est(1) - x_true(1);
        ey = x_est(2) - x_true(2);
        eth = wrap_angle(x_est(3) - x_true(3));
        sumPos2 = sumPos2 + ex^2 + ey^2;
        sumTh2  = sumTh2 + eth^2;
    end

    rmsPos = sqrt(sumPos2 / N);
    rmsTh  = sqrt(sumTh2  / N);

    fprintf('RMS position error [m]: %.4f\n', rmsPos);
    fprintf('RMS heading error [rad]: %.4f\n', rmsTh);
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function xnext = motion_model(x, u, dt)
    px = x(1); py = x(2); th = x(3);
    v = u(1);  w  = u(2);
    xnext = zeros(3,1);
    xnext(1) = px + dt*v*cos(th);
    xnext(2) = py + dt*v*sin(th);
    xnext(3) = wrap_angle(th + dt*w);
end

function F = jacobian_F(x, u, dt)
    th = x(3);
    v  = u(1);
    F = eye(3);
    F(1,3) = -dt*v*sin(th);
    F(2,3) =  dt*v*cos(th);
end

function L = jacobian_L(x, dt)
    th = x(3);
    L = zeros(3,2);
    L(1,1) = dt*cos(th);
    L(2,1) = dt*sin(th);
    L(3,2) = dt;
end

function z = meas_model(x, lm)
    px = x(1); py = x(2); th = x(3);
    mx = lm(1); my = lm(2);
    dx = mx - px;
    dy = my - py;
    r  = sqrt(dx^2 + dy^2);
    b  = wrap_angle(atan2(dy, dx) - th);
    z = [r; b];
end

function H = jacobian_H(x, lm)
    px = x(1); py = x(2);
    mx = lm(1); my = lm(2);
    dx = mx - px;
    dy = my - py;
    q  = dx^2 + dy^2;
    eps = 1e-12;
    if q < eps, q = eps; end
    r  = sqrt(q);

    H = zeros(2,3);
    % range
    H(1,1) = -dx/r;
    H(1,2) = -dy/r;
    % bearing
    H(2,1) =  dy/q;
    H(2,2) = -dx/q;
    H(2,3) = -1;
end

function idx = nearest_landmark(x, landmarks)
    dx = landmarks(:,1) - x(1);
    dy = landmarks(:,2) - x(2);
    d  = sqrt(dx.^2 + dy.^2);
    [~, idx] = min(d);
end
