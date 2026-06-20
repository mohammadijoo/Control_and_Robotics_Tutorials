% Chapter7_Lesson5.m
% EKF/UKF localization lab: Wheel + IMU + GPS (2D ground robot)
% Requires only base MATLAB. Optional: Sensor Fusion and Tracking Toolbox for comparison.

clear; clc; rng(7);

T = 80.0;
dt_imu = 0.01;
dt_wheel = 0.05;
dt_gps = 1.0;

[t, Xtrue, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, params] = sim_data(T, dt_imu, dt_wheel, dt_gps);

% State: [x; y; theta; v; bg; ba]
x0 = [0.5; -1.0; 0.0; 0.5; 0.0; 0.0];
P0 = diag([4.0, 4.0, (20*pi/180)^2, 1.0, 0.05^2, 0.2^2]);

% Noise specs
sigma_g = params.sigma_g;
sigma_a = params.sigma_a;
sigma_bg_rw = params.sigma_bg_rw;
sigma_ba_rw = params.sigma_ba_rw;

sigma_vw = params.sigma_vw;
sigma_ww = params.sigma_ww;
sigma_gps = params.sigma_gps;

Rw = diag([sigma_vw^2, sigma_ww^2]);
Rg = diag([sigma_gps^2, sigma_gps^2]);

% EKF uses continuous Qc for [n_g; n_a; w_bg; w_ba]
Qc = diag([sigma_g^2, sigma_a^2, sigma_bg_rw^2, sigma_ba_rw^2]);

% UKF uses a simple discrete Qd (tuned similarly)
Qd = diag([0, 0, (sigma_g*dt_imu)^2, (sigma_a*dt_imu)^2, (sigma_bg_rw*sqrt(dt_imu))^2, (sigma_ba_rw*sqrt(dt_imu))^2]);

% Run EKF and UKF
[xE, PE, XE] = run_ekf(t, x0, P0, Qc, Rw, Rg, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, dt_imu);
[xU, PU, XU] = run_ukf(t, x0, P0, Qd, Rw, Rg, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, dt_imu);

% Metrics
pos_rmse_E = sqrt(mean(sum((XE(:,1:2)-Xtrue(:,1:2)).^2,2)));
pos_rmse_U = sqrt(mean(sum((XU(:,1:2)-Xtrue(:,1:2)).^2,2)));
th_rmse_E = sqrt(mean(wrapToPi(XE(:,3)-Xtrue(:,3)).^2));
th_rmse_U = sqrt(mean(wrapToPi(XU(:,3)-Xtrue(:,3)).^2));

fprintf('Position RMSE (m): EKF=%g  UKF=%g\n', pos_rmse_E, pos_rmse_U);
fprintf('Heading RMSE (rad): EKF=%g  UKF=%g\n', th_rmse_E, th_rmse_U);

figure; plot(Xtrue(:,1), Xtrue(:,2), 'k', 'LineWidth',1.5); hold on;
plot(XE(:,1), XE(:,2), '--'); plot(XU(:,1), XU(:,2), ':');
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
legend('true','EKF','UKF'); title('Trajectory');

% --------------------------- Simulink hook (optional) ---------------------------
% You can programmatically build a simple Simulink model that implements the EKF
% predict/update as MATLAB Function blocks. This is minimal and intended as a starting point.
%
% create_simulink_model('Chapter7_Lesson5_Simulink');

% --------------------------- Functions ---------------------------

function [t, Xtrue, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, params] = sim_data(T, dt_imu, dt_wheel, dt_gps)
    n = floor(T/dt_imu) + 1;
    t = (0:n-1)' * dt_imu;

    sigma_g = 0.02;
    sigma_a = 0.20;
    sigma_bg_rw = 5e-4;
    sigma_ba_rw = 2e-3;
    sigma_vw = 0.10;
    sigma_ww = 0.02;
    sigma_gps = 1.5;

    Xtrue = zeros(n,6);
    Xtrue(1,:) = [0.0, 0.0, 0.2, 1.0, 0.02, -0.05];

    omega_true = zeros(n,1);
    a_true = zeros(n,1);

    for k = 1:n-1
        tt = t(k);
        omega_cmd = 0.25*sin(0.2*tt) + 0.05*sin(1.1*tt);
        a_cmd = 0.4*sin(0.1*tt);

        omega_true(k) = omega_cmd;
        a_true(k) = a_cmd;

        x = Xtrue(k,1); y = Xtrue(k,2); th = Xtrue(k,3); v = Xtrue(k,4); bg = Xtrue(k,5); ba = Xtrue(k,6);

        bg_next = bg + sigma_bg_rw*sqrt(dt_imu)*randn();
        ba_next = ba + sigma_ba_rw*sqrt(dt_imu)*randn();

        th_next = wrapToPi(th + omega_cmd*dt_imu);
        v_next  = v + a_cmd*dt_imu;
        x_next  = x + v*dt_imu*cos(th);
        y_next  = y + v*dt_imu*sin(th);

        Xtrue(k+1,:) = [x_next, y_next, th_next, v_next, bg_next, ba_next];
    end
    omega_true(end) = omega_true(end-1);
    a_true(end) = a_true(end-1);

    imu_omega_m = omega_true + Xtrue(:,5) + sigma_g*randn(n,1);
    imu_a_m = a_true + Xtrue(:,6) + sigma_a*randn(n,1);

    step_wheel = max(1, round(dt_wheel/dt_imu));
    wheel_idx = (1:step_wheel:n)';
    wheel_v = Xtrue(wheel_idx,4) + sigma_vw*randn(length(wheel_idx),1);
    wheel_omega = omega_true(wheel_idx) + sigma_ww*randn(length(wheel_idx),1);

    step_gps = max(1, round(dt_gps/dt_imu));
    gps_idx = (1:step_gps:n)';
    gps_xy = Xtrue(gps_idx,1:2) + sigma_gps*randn(length(gps_idx),2);

    params = struct('sigma_g',sigma_g,'sigma_a',sigma_a,'sigma_bg_rw',sigma_bg_rw,'sigma_ba_rw',sigma_ba_rw,...
                    'sigma_vw',sigma_vw,'sigma_ww',sigma_ww,'sigma_gps',sigma_gps);
end

function [x, P, Xlog] = run_ekf(t, x0, P0, Qc, Rw, Rg, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, dt)
    n = length(t);
    x = x0; P = P0;
    Xlog = zeros(n,6);

    wheel_map = containers.Map('KeyType','int32','ValueType','int32');
    for i=1:length(wheel_idx), wheel_map(wheel_idx(i)) = i; end
    gps_map = containers.Map('KeyType','int32','ValueType','int32');
    for i=1:length(gps_idx), gps_map(gps_idx(i)) = i; end

    for k=1:n
        omega_m = imu_omega_m(k);
        a_m = imu_a_m(k);

        % Predict
        [x, F, G] = f_disc(x, omega_m, a_m, dt);
        Qd = G*Qc*G';
        P = F*P*F' + Qd;

        % Wheel update
        if isKey(wheel_map,k)
            j = wheel_map(k);
            z = [wheel_v(j); wheel_omega(j)];
            h = [x(4); omega_m - x(5)];
            H = zeros(2,6); H(1,4)=1; H(2,5)=-1;
            [x,P] = kf_update(x,P,z,h,H,Rw);
        end

        % GPS update
        if isKey(gps_map,k)
            j = gps_map(k);
            z = gps_xy(j,:)';
            h = x(1:2);
            H = zeros(2,6); H(1,1)=1; H(2,2)=1;
            [x,P] = kf_update(x,P,z,h,H,Rg);
        end

        x(3) = wrapToPi(x(3));
        Xlog(k,:) = x';
    end
end

function [x, P, Xlog] = run_ukf(t, x0, P0, Qd, Rw, Rg, imu_omega_m, imu_a_m, wheel_idx, wheel_v, wheel_omega, gps_idx, gps_xy, dt)
    nT = length(t);
    x = x0; P = P0;
    Xlog = zeros(nT,6);

    alpha = 1e-3; beta = 2; kappa = 0;
    n = 6;
    lambda = alpha^2*(n+kappa) - n;

    Wm = ones(2*n+1,1) * (1/(2*(n+lambda)));
    Wc = Wm;
    Wm(1) = lambda/(n+lambda);
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);

    wheel_map = containers.Map('KeyType','int32','ValueType','int32');
    for i=1:length(wheel_idx), wheel_map(wheel_idx(i)) = i; end
    gps_map = containers.Map('KeyType','int32','ValueType','int32');
    for i=1:length(gps_idx), gps_map(gps_idx(i)) = i; end

    for k=1:nT
        omega_m = imu_omega_m(k);
        a_m = imu_a_m(k);

        % Predict
        Xsig = sigma_points(x,P,lambda);
        Xp = zeros(2*n+1,n);
        for i=1:(2*n+1)
            xi = Xsig(i,:)';
            Xp(i,:) = f_disc_state(xi, omega_m, a_m, dt)';
        end
        x = sum(Wm.*Xp,1)';
        x(3) = circ_mean(Xp(:,3), Wm);

        P = zeros(n,n);
        for i=1:(2*n+1)
            dx = (Xp(i,:)' - x);
            dx(3) = wrapToPi(dx(3));
            P = P + Wc(i) * (dx*dx');
        end
        P = P + Qd;

        % Wheel update
        if isKey(wheel_map,k)
            j = wheel_map(k);
            z = [wheel_v(j); wheel_omega(j)];
            [x,P] = ukf_update(x,P,z,@(s)[s(4); omega_m - s(5)], Rw, lambda, Wm, Wc);
        end

        % GPS update
        if isKey(gps_map,k)
            j = gps_map(k);
            z = gps_xy(j,:)';
            [x,P] = ukf_update(x,P,z,@(s)s(1:2), Rg, lambda, Wm, Wc);
        end

        x(3) = wrapToPi(x(3));
        Xlog(k,:) = x';
    end
end

function [x, F, G] = f_disc(x, omega_m, a_m, dt)
    % x = [x;y;theta;v;bg;ba]
    X = x(1); Y = x(2); th = x(3); v = x(4); bg = x(5); ba = x(6);
    omega = omega_m - bg;
    acc = a_m - ba;

    x(1) = X + v*dt*cos(th);
    x(2) = Y + v*dt*sin(th);
    x(3) = wrapToPi(th + omega*dt);
    x(4) = v + acc*dt;
    % bg, ba constant in mean

    F = eye(6);
    F(1,3) = -v*dt*sin(th);
    F(1,4) =  dt*cos(th);
    F(2,3) =  v*dt*cos(th);
    F(2,4) =  dt*sin(th);
    F(3,5) = -dt;
    F(4,6) = -dt;

    G = zeros(6,4);
    G(3,1) = dt;
    G(4,2) = dt;
    G(5,3) = sqrt(dt);
    G(6,4) = sqrt(dt);
end

function x1 = f_disc_state(x, omega_m, a_m, dt)
    [x1,~,~] = f_disc(x, omega_m, a_m, dt);
end

function [x,P] = kf_update(x,P,z,h,H,R)
    y = z - h;
    S = H*P*H' + R;
    K = P*H'/S;
    x = x + K*y;
    I = eye(size(P));
    P = (I - K*H)*P*(I - K*H)' + K*R*K'; % Joseph form
end

function Xsig = sigma_points(x,P,lambda)
    n = length(x);
    S = chol((n+lambda)*P, 'lower');
    Xsig = zeros(2*n+1,n);
    Xsig(1,:) = x';
    for i=1:n
        Xsig(1+i,:)   = (x + S(:,i))';
        Xsig(1+i+n,:) = (x - S(:,i))';
    end
    Xsig(:,3) = wrapToPi(Xsig(:,3));
end

function mu = circ_mean(theta, w)
    s = sum(w .* sin(theta));
    c = sum(w .* cos(theta));
    mu = atan2(s,c);
end

function [x,P] = ukf_update(x,P,z,hfun,R,lambda,Wm,Wc)
    n = length(x);
    Xsig = sigma_points(x,P,lambda);
    m = length(z);
    Zsig = zeros(2*n+1,m);
    for i=1:(2*n+1)
        Zsig(i,:) = hfun(Xsig(i,:)')';
    end
    zbar = sum(Wm.*Zsig,1)';
    S = zeros(m,m);
    Pxz = zeros(n,m);
    for i=1:(2*n+1)
        dz = (Zsig(i,:)' - zbar);
        dx = (Xsig(i,:)' - x);
        dx(3) = wrapToPi(dx(3));
        S = S + Wc(i) * (dz*dz');
        Pxz = Pxz + Wc(i) * (dx*dz');
    end
    S = S + R;
    K = Pxz / S;
    x = x + K*(z - zbar);
    x(3) = wrapToPi(x(3));
    P = P - K*S*K';
end

function create_simulink_model(modelName)
    new_system(modelName);
    open_system(modelName);
    % Add blocks as needed:
    % - Sources for IMU and wheel/GPS measurements (From Workspace)
    % - MATLAB Function blocks for predict and update
    % - Memory blocks for state/covariance
    % For a full model, students should connect blocks according to the
    % predict-update sequence shown in the lesson text.
    save_system(modelName);
end
