% Robot joint parameters
Tp = 0.15;
Kp_proc = 1.0;
u_min = -5; u_max = 5;

Kp = 8; Ki = 20; T_aw = 0.05;

r = 1.0;

odefun = @(t, x) joint_ode(t, x, r, Kp, Ki, T_aw, ...
                           Tp, Kp_proc, u_min, u_max);

x0 = [0; 0]; % [y; xI]
[t, x] = ode45(odefun, [0 1.5], x0);

y = x(:, 1);

function dx = joint_ode(t, x, r, Kp, Ki, T_aw, Tp, Kp_proc, u_min, u_max)
    y  = x(1);
    xI = x(2);

    e = r - y;
    v = Kp * e + Ki * xI;

    % Saturation
    u = min(max(v, u_min), u_max);

    % Back-calculation anti-windup
    e_aw = u - v;
    dxI = e + (1 / T_aw) * e_aw;

    % First-order plant
    dy = (-1 / Tp) * y + (Kp_proc / Tp) * u;

    dx = [dy; dxI];
end
