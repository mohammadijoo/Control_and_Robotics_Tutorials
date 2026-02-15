
Ts = 0.002;
Kp = 50;
Kd = 2;
tFinal = 5.0;

% Robot interface: implement these as needed
readJoint  = @() deal(0.0, 0.0);  % [q, dq]
writeTorque = @(tau) [];          % send torque

t0 = tic;
lastTime = 0.0;
nextTime = Ts;

e_prev = 0.0;

log_t = [];
log_Tk = [];
log_jitter = [];
log_q = [];
log_qd = [];
log_e = [];

while true
    now = toc(t0);
    if now >= tFinal
        break;
    end

    Tk = now - lastTime;
    lastTime = now;
    jitter = Tk - Ts;

    [q, dq] = readJoint();

    qd  = 0.5 * sin(2*pi*0.5*now);
    dqd = 0.5 * 2*pi*0.5 * cos(2*pi*0.5*now);

    e  = qd - q;
    de = (e - e_prev) / max(Tk, 1e-6);
    e_prev = e;

    tau = Kp * e + Kd * de;
    writeTorque(tau);

    log_t(end+1,1) = now;
    log_Tk(end+1,1) = Tk;
    log_jitter(end+1,1) = jitter;
    log_q(end+1,1) = q;
    log_qd(end+1,1) = qd;
    log_e(end+1,1) = e;

    % Sleep to enforce nominal Ts
    pause(max(0.0, nextTime - now));
    nextTime = nextTime + Ts;
end

% Plot log_Tk, log_jitter, and log_e vs log_t
