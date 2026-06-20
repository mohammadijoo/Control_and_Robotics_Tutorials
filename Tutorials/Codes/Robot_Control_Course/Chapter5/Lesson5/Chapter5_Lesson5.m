
% Parameters
m      = 1.0;
k_env  = 5000.0;
F_safe = 80.0;
a_brake = 30.0;
v_max  = 1.0;

v_force = F_safe / sqrt(m * k_env);

h = 5e-4;
T = 1.0;
N = floor(T / h);

x_target = 0.0;
k_p = 50.0;
k_d = 5.0;
k_v = 200.0;

x = 0.25;
v = 0.0;

xs = zeros(N,1);
vs = zeros(N,1);
Fs = zeros(N,1);
ts = (0:N-1)' * h;

for k = 1:N
    d = x;
    v_des = -k_p * (x - x_target) - k_d * v;

    v_brake = sqrt(max(0.0, 2.0 * a_brake * d));
    v_safe_bound = min([v_force, v_brake, v_max]);

    if v_des < -v_safe_bound
        v_safe_cmd = -v_safe_bound;
    else
        v_safe_cmd = v_des;
    end

    u = m * k_v * (v_safe_cmd - v);

    v = v + (h / m) * u;
    x = x + h * v;

    F = 0.0;
    if x < 0.0
        F = -k_env * x;
    end

    xs(k) = x;
    vs(k) = v;
    Fs(k) = F;
end

fprintf("Max penetration (m): %g\n", min(xs));
fprintf("Max contact force (N): %g\n", max(abs(Fs)));
fprintf("Theoretical bound F_safe (N): %g\n", F_safe);

figure;
subplot(3,1,1); plot(ts, xs); ylabel('x [m]');
subplot(3,1,2); plot(ts, vs); ylabel('v [m/s]');
subplot(3,1,3); plot(ts, Fs); ylabel('F [N]'); xlabel('t [s]');
