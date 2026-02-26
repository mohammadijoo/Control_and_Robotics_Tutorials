% Chapter 11 - SLAM I (Filter-Based SLAM)
% Lesson 2: EKF-SLAM (structure and limitations)
%
% This file provides:
%  1) A minimal EKF-SLAM loop (2D unicycle + range-bearing landmarks).
%  2) A simple programmatic Simulink construction that calls MATLAB Function
%     blocks for Predict/Update (illustrative; no .slx is shipped).
%
% Notes:
% - Known data association for simplicity.
% - Angles are wrapped to (-pi, pi].

function Chapter11_Lesson2()
    rng(2);

    % Ground-truth landmarks
    L = [5 1; 2 6; 8 7; 10 2.5];   % [x y]
    nL = size(L,1);

    % Noise models
    Q = diag([0.05^2, deg2rad(2)^2]);     % control noise for [v w]
    R = diag([0.15^2, deg2rad(3)^2]);     % measurement noise for [range bearing]

    % EKF-SLAM state
    mu = zeros(3,1);          % start pose
    P  = eye(3) * 1e-6;
    id_to_slot = containers.Map('KeyType','int32','ValueType','int32');

    % Ground truth
    gt = [0;0;0];

    dt = 0.1;
    steps = 200;

    traj_gt  = zeros(steps+1,3);
    traj_est = zeros(steps+1,3);
    traj_gt(1,:)  = gt';
    traj_est(1,:) = mu(1:3)';

    for k = 1:steps
        v_cmd = 0.7;
        w_cmd = 0.15 * sin(0.05*k);

        % Noisy motion for ground truth
        v_noisy = v_cmd + sqrt(Q(1,1))*randn();
        w_noisy = w_cmd + sqrt(Q(2,2))*randn();

        gt = unicycle_step(gt, v_noisy, w_noisy, dt);

        % EKF prediction uses commanded control
        [mu, P] = ekf_predict(mu, P, v_cmd, w_cmd, dt, Q);

        % Measurements
        for id = 1:nL
            dx = L(id,1) - gt(1);
            dy = L(id,2) - gt(2);
            rng_true = hypot(dx,dy);
            if rng_true < 7.0
                brg_true = wrap_angle(atan2(dy,dx) - gt(3));
                z = [rng_true + sqrt(R(1,1))*randn();
                     wrap_angle(brg_true + sqrt(R(2,2))*randn())];

                if ~isKey(id_to_slot, id)
                    [mu, P, id_to_slot] = init_landmark(mu, P, id_to_slot, id, z, R);
                end
                [mu, P] = ekf_update(mu, P, id_to_slot, id, z, R);
            end
        end

        traj_gt(k+1,:)  = gt';
        traj_est(k+1,:) = mu(1:3)';
    end

    pos_err = norm(traj_gt(end,1:2) - traj_est(end,1:2));
    ang_err = abs(wrap_angle(traj_gt(end,3) - traj_est(end,3)));
    fprintf('Final position error: %.3f\n', pos_err);
    fprintf('Final angle error (deg): %.3f\n', rad2deg(ang_err));

    figure; hold on; grid on; axis equal;
    plot(traj_gt(:,1), traj_gt(:,2), 'LineWidth', 1.5);
    plot(traj_est(:,1), traj_est(:,2), 'LineWidth', 1.5);
    plot(L(:,1), L(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
    legend('ground truth','EKF-SLAM estimate','landmarks');
    title('EKF-SLAM demo (known association)');

    % Optional: build an illustrative Simulink model
    % build_simulink_skeleton();
end

% -------------------------- Core Functions --------------------------

function x = unicycle_step(x, v, w, dt)
    th = x(3);
    if abs(w) < 1e-9
        x(1) = x(1) + v*dt*cos(th);
        x(2) = x(2) + v*dt*sin(th);
    else
        x(1) = x(1) + (v/w)*(sin(th+w*dt) - sin(th));
        x(2) = x(2) - (v/w)*(cos(th+w*dt) - cos(th));
        x(3) = x(3) + w*dt;
    end
    x(3) = wrap_angle(x(3));
end

function [mu, P] = ekf_predict(mu, P, v, w, dt, Q)
    % Predict robot pose; landmarks static
    x = mu(1); y = mu(2); th = mu(3);

    if abs(w) < 1e-9
        x_new = x + v*dt*cos(th);
        y_new = y + v*dt*sin(th);
        th_new = th;
    else
        x_new = x + (v/w)*(sin(th+w*dt) - sin(th));
        y_new = y - (v/w)*(cos(th+w*dt) - cos(th));
        th_new = th + w*dt;
    end
    th_new = wrap_angle(th_new);

    mu(1:3) = [x_new; y_new; th_new];

    D = length(mu);
    F = eye(D);
    Gu = zeros(D,2);

    if abs(w) < 1e-9
        F(1,3) = -v*dt*sin(th);
        F(2,3) =  v*dt*cos(th);
        Gu(1,1) = dt*cos(th);
        Gu(2,1) = dt*sin(th);
        Gu(3,2) = dt;
    else
        F(1,3) = (v/w)*(cos(th+w*dt) - cos(th));
        F(2,3) = (v/w)*(sin(th+w*dt) - sin(th));
        Gu(1,1) = (1/w)*(sin(th+w*dt) - sin(th));
        Gu(2,1) = -(1/w)*(cos(th+w*dt) - cos(th));
        Gu(3,2) = dt;
        Gu(1,2) = (v/(w^2))*(sin(th) - sin(th+w*dt)) + (v/w)*(dt*cos(th+w*dt));
        Gu(2,2) = (v/(w^2))*(cos(th+w*dt) - cos(th)) + (v/w)*(dt*sin(th+w*dt));
    end

    P = F*P*F' + Gu*Q*Gu';
    P = 0.5*(P + P');
end

function [mu, P] = ekf_update(mu, P, id_to_slot, id, z, R)
    [zhat, H] = expected_and_jacobian(mu, id_to_slot, id);
    innov = z - zhat;
    innov(2) = wrap_angle(innov(2));

    S = H*P*H' + R;
    K = P*H'/S;

    mu = mu + K*innov;
    mu(3) = wrap_angle(mu(3));

    D = length(mu);
    I = eye(D);
    % Joseph form
    P = (I-K*H)*P*(I-K*H)' + K*R*K';
    P = 0.5*(P + P');
end

function [zhat, H] = expected_and_jacobian(mu, id_to_slot, id)
    D = length(mu);
    H = zeros(2,D);

    x = mu(1); y = mu(2); th = mu(3);
    slot = id_to_slot(id);
    idx = 3 + 2*slot + 1; % MATLAB 1-based
    mx = mu(idx);
    my = mu(idx+1);

    dx = mx - x; dy = my - y;
    q = dx^2 + dy^2;
    r = sqrt(q);

    zhat = [r; wrap_angle(atan2(dy,dx) - th)];

    % range
    H(1,1) = -dx/r; H(1,2) = -dy/r;
    H(1,idx) = dx/r; H(1,idx+1) = dy/r;

    % bearing
    H(2,1) = dy/q;  H(2,2) = -dx/q; H(2,3) = -1;
    H(2,idx) = -dy/q; H(2,idx+1) = dx/q;
end

function [mu, P, id_to_slot] = init_landmark(mu, P, id_to_slot, id, z, R)
    x = mu(1); y = mu(2); th = mu(3);
    r = z(1); b = z(2);
    ang = th + b;

    mx = x + r*cos(ang);
    my = y + r*sin(ang);
    m = [mx; my];

    Gx = [1 0 -r*sin(ang);
          0 1  r*cos(ang)];
    Gz = [cos(ang) -r*sin(ang);
          sin(ang)  r*cos(ang)];

    D = length(mu);
    Prr = P(1:3,1:3);
    Pxr = P(:,1:3);

    Pmm = Gx*Prr*Gx' + Gz*R*Gz';
    Pxm = Pxr*Gx';

    % Augment state
    mu = [mu; m];
    Pnew = zeros(D+2, D+2);
    Pnew(1:D,1:D) = P;
    Pnew(1:D,D+1:D+2) = Pxm;
    Pnew(D+1:D+2,1:D) = Pxm';
    Pnew(D+1:D+2,D+1:D+2) = Pmm;
    P = 0.5*(Pnew + Pnew');

    id_to_slot(id) = id_to_slot.Count; % new last slot (0-based)
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

% ---------------------- Simulink Skeleton (Optional) ----------------------
function build_simulink_skeleton()
    % Illustrative: builds a model with two MATLAB Function blocks.
    % You can wire sensors/controls later. This is a *structure* template.
    mdl = 'EKFSLAM_Skeleton';
    if bdIsLoaded(mdl); close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Predict']);
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Update']);
    set_param([mdl '/Predict'], 'Position', [100 100 250 180]);
    set_param([mdl '/Update'],  'Position', [350 100 500 180]);

    % Users typically add Bus signals for mu and P, and feed v,w,dt,Q plus z,R.
    save_system(mdl);
    fprintf('Created Simulink model: %s.slx (structure only)\n', mdl);
end
