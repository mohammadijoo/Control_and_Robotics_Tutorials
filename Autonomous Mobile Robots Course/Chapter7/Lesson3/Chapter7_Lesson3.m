% Chapter7_Lesson3.m
% Consistency and Linearization Pitfalls — EKF + NEES/NIS (MATLAB/Simulink-friendly).
%
% This script:
%   1) Simulates a planar robot x=[px; py; theta] with unicycle kinematics.
%   2) Uses range-bearing measurements to known landmarks.
%   3) Runs EKF and reports ANEES and ANIS with chi-square 95% bounds.
%
% Simulink note:
%   - The functions f_motion(), h_meas(), jacF(), jacH(), and ekf_step()
%     can be pasted into MATLAB Function blocks (one for predict, one for update),
%     and driven by Simulink signals (u, z, dt).
%
% Required toolboxes: none (Statistics Toolbox optional for chi2inv).
clear; clc;

rng(1);

% --- configuration ---
steps  = 200;
trials = 40;
alpha  = 0.05;

landmarks = [5 0; 0 5; 5 5];

Q = diag([0.08^2, 0.08^2, (3*pi/180)^2]);
R = diag([0.15^2, (2*pi/180)^2]);

dts = [0.05, 0.20];

for dt = dts
    nees_sum = 0; nis_sum = 0; count = 0;

    for tr = 1:trials
        x_true = [0; 0; 0];

        x_hat  = [0.5; -0.4; 10*pi/180];
        P      = diag([0.8^2, 0.8^2, (15*pi/180)^2]);

        for k = 1:steps
            v  = 1.0 + 0.2*sin(0.05*(k-1));
            om = 0.35 + 0.25*sin(0.03*(k-1));
            u  = [v; om];

            % true propagation with noise
            w = chol(Q,'lower') * randn(3,1);
            x_true = f_motion(x_true, u, dt) + w;
            x_true(3) = wrap_angle(x_true(3));

            lm = landmarks(mod(k-1,size(landmarks,1))+1, :)';

            % measurement
            z = h_meas(x_true, lm) + chol(R,'lower') * randn(2,1);
            z(2) = wrap_angle(z(2));

            % EKF step
            [x_hat, P, nu, S] = ekf_step(x_hat, P, u, z, dt, Q, R, lm);

            % NEES/NIS
            e = x_true - x_hat;
            e(3) = wrap_angle(e(3));

            nees = e'*(P\e);
            nis  = nu'*(S\nu);

            nees_sum = nees_sum + nees;
            nis_sum  = nis_sum  + nis;
            count = count + 1;
        end
    end

    ANEES = nees_sum / count;
    ANIS  = nis_sum  / count;

    fprintf('\n=== dt=%.3f ===\n', dt);
    fprintf('EKF ANEES=%.3f (expected ~ 3)\n', ANEES);
    fprintf('EKF ANIS =%.3f (expected ~ 2)\n', ANIS);

    if exist('chi2inv','file') == 2
        lo_nees = chi2inv(alpha/2, count*3)/count;
        hi_nees = chi2inv(1-alpha/2, count*3)/count;
        lo_nis  = chi2inv(alpha/2, count*2)/count;
        hi_nis  = chi2inv(1-alpha/2, count*2)/count;

        fprintf('95%% bounds for ANEES: [%.3f, %.3f]\n', lo_nees, hi_nees);
        fprintf('95%% bounds for ANIS : [%.3f, %.3f]\n', lo_nis, hi_nis);
    else
        fprintf('chi2inv not available; install Statistics and Machine Learning Toolbox for bounds.\n');
    end

    fprintf('Interpretation: values above upper bounds indicate overconfidence (inconsistency).\n');
end

% ---------------- local functions ----------------

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi);
    if a < 0, a = a + 2*pi; end
    a = a - pi;
end

function xn = f_motion(x, u, dt)
    px = x(1); py = x(2); th = x(3);
    v = u(1); om = u(2);
    xn = [px + v*dt*cos(th);
          py + v*dt*sin(th);
          wrap_angle(th + om*dt)];
end

function F = jacF(x, u, dt)
    th = x(3); v = u(1);
    F = eye(3);
    F(1,3) = -v*dt*sin(th);
    F(2,3) =  v*dt*cos(th);
end

function z = h_meas(x, lm)
    px = x(1); py = x(2); th = x(3);
    dx = lm(1) - px;
    dy = lm(2) - py;
    r  = hypot(dx, dy);
    b  = wrap_angle(atan2(dy, dx) - th);
    z = [r; b];
end

function H = jacH(x, lm)
    px = x(1); py = x(2);
    dx = lm(1) - px;
    dy = lm(2) - py;
    r2 = dx^2 + dy^2;
    r  = sqrt(max(r2, 1e-12));
    H = zeros(2,3);
    H(1,1) = -dx/r;
    H(1,2) = -dy/r;
    H(2,1) =  dy/r2;
    H(2,2) = -dx/r2;
    H(2,3) = -1;
end

function [x_upd, P_upd, nu, S] = ekf_step(x, P, u, z, dt, Q, R, lm)
    % Predict
    F = jacF(x, u, dt);
    x_pred = f_motion(x, u, dt);
    P_pred = F*P*F' + Q;

    % Update
    H = jacH(x_pred, lm);
    z_pred = h_meas(x_pred, lm);
    nu = z - z_pred;
    nu(2) = wrap_angle(nu(2));

    S = H*P_pred*H' + R;
    K = P_pred*H'/S;

    x_upd = x_pred + K*nu;
    x_upd(3) = wrap_angle(x_upd(3));

    I = eye(3);
    P_upd = (I - K*H)*P_pred*(I - K*H)' + K*R*K'; % Joseph form
end
