
% Parameters
m_true = 1.2; b_true = 0.3; g_true = 9.81;
m_nom  = 1.0; b_nom  = 0.1; g_nom  = 9.81;
Kp = 25; Kd = 10;

dt = 0.002;
T  = 12.0;
t  = 0:dt:T;
N  = numel(t);

q  = 0;
qd = 0;

Z = zeros(N,4);
Y = zeros(N,1);

disturbance = @(q, qd) 0.4 * sin(2*q) + 0.05 * qd.^3;

for k = 1:N
    tk = t(k);
    A = 0.7; w = 0.7;
    qd_des    = A * sin(w * tk);
    qd_dotdes = A * w * cos(w * tk);
    qdd_des   = -A * w^2 * sin(w * tk);

    e    = q - qd_des;
    edot = qd - qd_dotdes;
    v    = qdd_des - Kd * edot - Kp * e;
    tau_nom = m_nom * v + b_nom * qd + g_nom * sin(q);

    d_true = disturbance(q, qd);
    qdd    = (tau_nom - b_true * qd - g_true * sin(q) - d_true) / m_true;

    % Euler integration
    qd = qd + dt * qdd;
    q  = q  + dt * qd;

    Z(k,:) = [q, qd, qd_des, qdd_des];
    Y(k)   = tau_nom - (m_nom * v + b_nom * qd + g_nom * sin(q)); % torque error
end

% Fit a GP residual model
gprMdl = fitrgp(Z, Y, "BasisFunction", "none", ...
                       "KernelFunction", "squaredexponential", ...
                       "Sigma", 1e-3);

% Save the model for Simulink
save("residualGP.mat", "gprMdl");
