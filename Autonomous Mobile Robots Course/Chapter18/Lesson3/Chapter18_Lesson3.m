% Chapter18_Lesson3.m
% Long-Range Localization Drift Handling (2D EKF)
% State: [x; y; psi; b_g]
% Features:
%   - Propagation with wheel speed + gyro
%   - GNSS/RTK updates (sequential scalar updates + gating)
%   - Terrain-aware process noise inflation
%   - Anchor re-localization updates
%   - Integrity monitor (covariance inflation)
%
% Simulink note:
%   The same logic maps naturally to a multi-rate model:
%   (1) Propagation subsystem at dt=0.1 s
%   (2) GNSS update subsystem at 1 Hz
%   (3) Anchor update trigger subsystem (event-based)

rng(18);

dt = 0.1;
N  = 1800;
bg_true = 0.015;

x_true = zeros(4, N);   % [x; y; psi; bg]
x_est  = [0; 0; 0.05; 0];
P_est  = diag([1, 1, deg2rad(5)^2, 0.03^2]);

x_hist = zeros(4, N);
P_trace = zeros(1, N);
nis_gnss = [];
nis_anchor = [];

for k = 2:N
    tk = (k-1) * dt;
    v_cmd = 1.5 + 0.3 * sin(0.015 * tk);
    w_cmd = 0.08 * sin(0.010 * tk) + 0.04 * sin(0.040 * tk);

    rough = ((k-1) >= 300 && (k-1) < 520) || ...
            ((k-1) >= 840 && (k-1) < 1100) || ...
            ((k-1) >= 1400 && (k-1) < 1620);

    sigma_w_true = ternary(rough, 0.03, 0.008);
    sigma_v_true = ternary(rough, 0.10, 0.02);

    v_true = v_cmd + sigma_v_true * randn;
    w_true = w_cmd + sigma_w_true * randn;

    x_true(1,k) = x_true(1,k-1) + v_true * dt * cos(x_true(3,k-1));
    x_true(2,k) = x_true(2,k-1) + v_true * dt * sin(x_true(3,k-1));
    x_true(3,k) = wrap_angle(x_true(3,k-1) + (w_true - bg_true) * dt);
    x_true(4,k) = bg_true;

    vm = v_true + ternary(rough, 0.12, 0.03) * randn;
    wm = w_true + ternary(rough, 0.05, 0.01) * randn;

    [x_est, P_est] = propagate_ekf(x_est, P_est, vm, wm, dt, rough);

    outage = (((k-1) >= 420 && (k-1) < 980) || ((k-1) >= 1250 && (k-1) < 1500));

    if mod(k-1, 10) == 0 && ~outage
        rtk_fixed = (~rough) && (mod(k-1, 90) ~= 0);
        sigma_gps = ternary(rtk_fixed, 0.15, 0.75);

        zgx = x_true(1,k) + sigma_gps * randn;
        zgy = x_true(2,k) + sigma_gps * randn;

        innov_x = zgx - x_est(1);
        Sx = P_est(1,1) + sigma_gps^2;
        d2x = innov_x^2 / Sx;
        if d2x < 6.63
            [x_est, P_est, nisx] = scalar_update(x_est, P_est, 1, zgx, sigma_gps^2, false);
            nis_gnss(end+1) = nisx; %#ok<AGROW>
        end

        innov_y = zgy - x_est(2);
        Sy = P_est(2,2) + sigma_gps^2;
        d2y = innov_y^2 / Sy;
        if d2y < 6.63
            [x_est, P_est, nisy] = scalar_update(x_est, P_est, 2, zgy, sigma_gps^2, false);
            nis_gnss(end+1) = nisy; %#ok<AGROW>
        end
    end

    if any((k-1) == [1000, 1510, 1710])
        zax = x_true(1,k) + 0.10 * randn;
        zay = x_true(2,k) + 0.10 * randn;
        zap = wrap_angle(x_true(3,k) + deg2rad(2) * randn);

        [x_est, P_est, n1] = scalar_update(x_est, P_est, 1, zax, 0.10^2, false);
        [x_est, P_est, n2] = scalar_update(x_est, P_est, 2, zay, 0.10^2, false);
        [x_est, P_est, n3] = scalar_update(x_est, P_est, 3, zap, deg2rad(2)^2, true);
        nis_anchor = [nis_anchor, n1, n2, n3]; %#ok<AGROW>
    end

    if numel(nis_gnss) >= 20
        if mean(nis_gnss(end-19:end)) > 1.8
            P_est = 1.02 * P_est;
        end
    end

    x_hist(:,k) = x_est;
    P_trace(k) = trace(P_est);
end

pos_err = x_hist(1:2,:) - x_true(1:2,:);
rmse = sqrt(mean(sum(pos_err.^2, 1)));
final_err = norm(pos_err(:,end));
final_heading_deg = rad2deg(wrap_angle(x_hist(3,end) - x_true(3,end)));

fprintf('=== Chapter18_Lesson3.m : Long-Range Localization Drift Handling ===\n');
fprintf('Mission duration: %.1f s\n', N*dt);
fprintf('Position RMSE: %.3f m\n', rmse);
fprintf('Final position error: %.3f m\n', final_err);
fprintf('Final heading error: %.3f deg\n', final_heading_deg);
fprintf('Mean scalar NIS (GNSS accepted): %.3f\n', mean_or_nan(nis_gnss));
fprintf('Mean scalar NIS (Anchor): %.3f\n', mean_or_nan(nis_anchor));
fprintf('Final covariance trace: %.5f\n', P_trace(end));

report_outage('Outage1', 421, 980, x_hist, x_true);
report_outage('Outage2', 1251, 1500, x_hist, x_true);

%% --- Local functions ---

function [x, P] = propagate_ekf(x, P, vm, wm, dt, rough)
    psi = x(3);
    bg  = x(4);

    x(1) = x(1) + vm * dt * cos(psi);
    x(2) = x(2) + vm * dt * sin(psi);
    x(3) = wrap_angle(x(3) + (wm - bg) * dt);

    F = eye(4);
    F(1,3) = -vm * dt * sin(psi);
    F(2,3) =  vm * dt * cos(psi);
    F(3,4) = -dt;

    sigma_v = ternary(rough, 0.12, 0.03);
    sigma_w = ternary(rough, 0.06, 0.01);
    sigma_bg = ternary(rough, 0.0030, 0.0008);

    Q = diag([(sigma_v*dt)^2, (sigma_v*dt)^2, (sigma_w*dt)^2, sigma_bg^2*dt]);

    P = F * P * F' + Q;
    P = 0.5 * (P + P');
end

function [x, P, nis] = scalar_update(x, P, idx, z, R, angleResidual)
    innov = z - x(idx);
    if angleResidual
        innov = wrap_angle(innov);
    end

    S = P(idx,idx) + R;
    K = P(:,idx) / S;

    x = x + K * innov;
    x(3) = wrap_angle(x(3));

    P = P - K * P(idx,:);
    P = 0.5 * (P + P');

    nis = innov^2 / S;
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function y = ternary(cond, a, b)
    if cond
        y = a;
    else
        y = b;
    end
end

function m = mean_or_nan(v)
    if isempty(v)
        m = NaN;
    else
        m = mean(v);
    end
end

function report_outage(name, a, b, x_hist, x_true)
    e_start = norm(x_hist(1:2, a) - x_true(1:2, a));
    e_end = norm(x_hist(1:2, b) - x_true(1:2, b));
    fprintf('%s: error at start=%.3f m, end=%.3f m, growth=%.3f m\n', ...
        name, e_start, e_end, e_end - e_start);
end
