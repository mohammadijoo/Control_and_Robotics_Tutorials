
% Parameters
J = 0.05;
K = 10.0;
D = 2.0;

% Simulation parameters
dt = 1e-3;
T  = 1.0;
N  = T / dt;

q   = 0.0;
qd  = 0.0;

ts  = zeros(N,1);
qs  = zeros(N,1);
qds = zeros(N,1);

for k = 1:N
    t = (k-1) * dt;

    if t >= 0.1
        q_ref = 0.5;
    else
        q_ref = 0.0;
    end
    qd_ref  = 0.0;
    qdd_ref = 0.0;

    if t >= 0.3 && t <= 0.35
        tau_ext = 0.2;
    else
        tau_ext = 0.0;
    end

    e  = q  - q_ref;
    ed = qd - qd_ref;

    tau_m = J * qdd_ref - K * e - D * ed;

    qdd = (tau_m + tau_ext) / J;

    qd = qd + dt * qdd;
    q  = q  + dt * qd;

    ts(k)  = t;
    qs(k)  = q;
    qds(k) = qd;
end

plot(ts, qs, ts, (ts >= 0.1) * 0.5);
legend('q(t)', 'q_d(t)');
xlabel('time [s]');
ylabel('position [rad]');
title('1-DOF Joint Impedance Control (MATLAB)');
