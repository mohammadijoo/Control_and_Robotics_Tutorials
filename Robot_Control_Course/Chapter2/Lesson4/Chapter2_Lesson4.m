
function robot_pd_tuning_example
    J = 0.5;
    B = 0.05;
    Mp_star = 0.1; % 10% overshoot
    Ts_star = 0.5; % 0.5 s settling time

    zeta = dampingRatioFromOvershoot(Mp_star);
    wn   = 4 / (zeta * Ts_star);
    Kp   = J * wn^2;
    Kd   = 2 * J * zeta * wn - B;
    Kd   = max(Kd, 0);

    fprintf("Kp = %.3f, Kd = %.3f\n", Kp, Kd);

    % Simulate closed-loop error dynamics for a unit step in qd
    qd = 1.0; % rad
    x0 = [0; 0]; % [e; de]

    tspan = [0 2];
    [t, x] = ode45(@(t, x) errorDynamics(t, x, J, B, Kp, Kd, qd), tspan, x0);

    e  = x(:,1);
    de = x(:,2);

    figure;
    plot(t, e);
    xlabel('Time [s]');
    ylabel('Position error e(t) [rad]');
    title('Closed-loop PD error dynamics');
end

function zeta = dampingRatioFromOvershoot(Mp_star)
    if Mp_star <= 0 || Mp_star >= 1
        error('Mp_star must be in (0, 1).');
    end
    logMp = log(Mp_star);
    zeta  = -logMp / sqrt(pi^2 + logMp^2);
end

function dx = errorDynamics(~, x, J, B, Kp, Kd, qd)
    e  = x(1);
    de = x(2);

    % tau = -Kp*(e) - Kd*(de), qd constant, so error dynamics:
    % J*ddot{e} + (B + Kd)*dot{e} + Kp*e = 0
    dde = -(B + Kd)/J * de - Kp/J * e;

    dx = [de; dde];
end
