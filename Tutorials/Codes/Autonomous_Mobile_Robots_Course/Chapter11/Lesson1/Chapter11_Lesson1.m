% Chapter11_Lesson1.m
% Autonomous Mobile Robots (Control Engineering)
% Chapter 11: SLAM I — Filter-Based SLAM
% Lesson 1: The SLAM Problem Formulation
%
% This file contains:
%   (A) A minimal EKF-SLAM predict/update for an augmented state.
%   (B) A helper that can (optionally) generate a simple Simulink model shell
%       that calls the EKF step inside a MATLAB Function block.
%
% Note: EKF-SLAM algorithmic details are expanded in Lesson 2; here we use EKF
% only to operationalize the Bayes recursion for filter-based SLAM.

function Chapter11_Lesson1()
    rng(7);

    % --- Landmarks (ground truth, unknown to filter) ---
    landmarks = [4 2; 8 -1; 2 -3];
    N = size(landmarks,1);

    % --- True state ---
    x_true = [0;0;0];

    % --- EKF-SLAM belief ---
    y = zeros(3+2*N,1);
    y(1:3) = [0;0;0];                       % pose mean
    y(4:end) = [3;1; 7;0; 1;-2];            % rough landmark guesses

    P = eye(length(y))*1e-3;
    for i=1:N
        idx = 3 + 2*(i-1) + 1;
        P(idx:idx+1, idx:idx+1) = eye(2)*4.0;
    end

    Q_pose = diag([0.02^2, 0.02^2, (deg2rad(1.0))^2]);
    R_meas = diag([0.10^2, (deg2rad(2.0))^2]);

    U = [1.0  0.10 1.0;
         1.0  0.00 1.0;
         1.0 -0.10 1.0;
         1.0  0.00 1.0];

    for t=1:size(U,1)
        u = U(t,:).';

        % True motion with process noise
        x_true = motion_model(x_true, u) + mvnrnd([0 0 0], Q_pose).';
        x_true(3) = wrap_angle(x_true(3));

        % Predict
        [y,P] = ekf_slam_predict(y,P,u,Q_pose);

        % Observe all landmarks (known association)
        for i=1:N
            z_true  = meas_model(x_true, landmarks(i,:).');
            z_noisy = z_true + mvnrnd([0 0], R_meas).';
            z_noisy(2) = wrap_angle(z_noisy(2));

            [y,P] = ekf_slam_update_landmark(y,P,z_noisy,i,R_meas);
        end

        fprintf('t=%d: pose mean = [%.3f %.3f %.3f]\\n', t, y(1), y(2), y(3));
    end
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end

function x2 = motion_model(x,u)
    px=x(1); py=x(2); th=x(3);
    v=u(1); w=u(2); dt=u(3);

    if abs(w) < 1e-12
        px2 = px + v*dt*cos(th);
        py2 = py + v*dt*sin(th);
        th2 = th;
    else
        thN = th + w*dt;
        px2 = px + (v/w)*(sin(thN)-sin(th));
        py2 = py - (v/w)*(cos(thN)-cos(th));
        th2 = thN;
    end
    x2 = [px2; py2; wrap_angle(th2)];
end

function Fx = jac_motion_wrt_state(x,u)
    th=x(3);
    v=u(1); w=u(2); dt=u(3);
    Fx = eye(3);

    if abs(w) < 1e-12
        Fx(1,3) = -v*dt*sin(th);
        Fx(2,3) =  v*dt*cos(th);
    else
        thN = th + w*dt;
        Fx(1,3) = (v/w)*(cos(thN)-cos(th));
        Fx(2,3) = (v/w)*(sin(thN)-sin(th));
    end
end

function z = meas_model(x, m)
    px=x(1); py=x(2); th=x(3);
    dx = m(1)-px;
    dy = m(2)-py;
    r = sqrt(dx^2 + dy^2);
    b = atan2(dy,dx) - th;
    z = [r; wrap_angle(b)];
end

function [Hx,Hm] = jac_meas_wrt_pose_landmark(x,m)
    px=x(1); py=x(2);
    dx = m(1)-px;
    dy = m(2)-py;
    q = dx^2 + dy^2;
    if q < 1e-12, q = 1e-12; end
    r = sqrt(q);

    Hx = [ -dx/r, -dy/r, 0;
            dy/q, -dx/q, -1 ];
    Hm = [  dx/r,  dy/r;
           -dy/q,  dx/q ];
end

function [y_pred,P_pred] = ekf_slam_predict(y,P,u,Q_pose)
    n = length(y);
    x = y(1:3);
    Fx = jac_motion_wrt_state(x,u);
    x_pred = motion_model(x,u);

    F = eye(n);
    F(1:3,1:3) = Fx;

    Q = zeros(n);
    Q(1:3,1:3) = Q_pose;

    y_pred = y;
    y_pred(1:3) = x_pred;
    P_pred = F*P*F.' + Q;
end

function [y_upd,P_upd] = ekf_slam_update_landmark(y,P,z,landmark_id,R_meas)
    n = length(y);
    i = landmark_id - 1;     % 0-based
    idx = 3 + 2*i + 1;
    x = y(1:3);
    mi = y(idx:idx+1);

    z_hat = meas_model(x,mi);
    innov = [z(1)-z_hat(1); wrap_angle(z(2)-z_hat(2))];

    [Hx,Hm] = jac_meas_wrt_pose_landmark(x,mi);

    H = zeros(2,n);
    H(:,1:3) = Hx;
    H(:,idx:idx+1) = Hm;

    S = H*P*H.' + R_meas;
    K = P*H.'/S;

    y_upd = y + K*innov;
    y_upd(3) = wrap_angle(y_upd(3));

    I = eye(n);
    P_upd = (I-K*H)*P*(I-K*H).' + K*R_meas*K.';
end

% Optional: generate a tiny Simulink shell (requires Simulink)
function build_simulink_shell()
    mdl = 'SLAM_BayesFilter_Shell';
    if bdIsLoaded(mdl); close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/u'], 'Value', '[1;0.1;1]');
    add_block('simulink/Sources/Constant', [mdl '/z'], 'Value', '[5;0]');
    add_block('simulink/Sources/Constant', [mdl '/R'], 'Value', 'diag([0.1^2,(deg2rad(2))^2])');
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/EKFStep']);
    add_block('simulink/Sinks/Display', [mdl '/Display']);

    set_param([mdl '/EKFStep'], 'Script', ...
        ['function y_out = f(y_in,P_in,u,z,R)\n' ...
         '% Call an EKF update/predict in MATLAB code. This is a shell; wire in states as needed.\n' ...
         'y_out = y_in; %#ok<NASGU>\n' ...
         'end\n']);

    add_line(mdl,'u/1','EKFStep/3');
    add_line(mdl,'z/1','EKFStep/4');
    add_line(mdl,'R/1','EKFStep/5');
    add_line(mdl,'EKFStep/1','Display/1');

    save_system(mdl);
end
