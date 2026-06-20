
% X_train: N x d matrix of inputs [q, dq, ddq_des, ...]
% y_train: N x 1 vector of torque residuals
load("training_data.mat","X_train","y_train");

gprMdl = fitrgp(X_train, y_train, ...
    "KernelFunction","squaredexponential", ...
    "KernelScale","auto", ...
    "Sigma",1e-3, ...
    "Standardize",true);

% Example: online control step (inside a control loop or Simulink MATLAB Function block)
function tau = gp_augmented_torque(q, dq, qd, dqd, ddqd)
    % Nominal computed-torque (for 1-DOF example)
    m_nom = 0.9;
    l_nom = 0.45;
    g = 9.81;
    kp = 80;
    kd = 20;

    e = qd - q;
    de = dqd - dq;
    v = ddqd + kd * de + kp * e;

    I0 = m_nom * l_nom^2;
    tau_nom = I0 * v + m_nom * g * l_nom * sin(q);

    % GP input vector must match training features
    x = [q, dq, ddqd];
    d_hat = predict(gprMdl, x);

    tau = tau_nom + d_hat;
end
