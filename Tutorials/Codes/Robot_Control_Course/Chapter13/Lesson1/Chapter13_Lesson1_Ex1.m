
dt = 1e-3;
T = 2.0;
N = round(T / dt);

x = zeros(1, N+1);
u_nom = zeros(1, N);
u_safe = zeros(1, N);

x(1) = 0.05;
x_ref = -1.0;
k_p = 2.0;
gamma = 5.0;

for k = 1:N
    u_nom(k) = -k_p * (x(k) - x_ref);
    u_safe(k) = cbf_filter_1d(x(k), u_nom(k), gamma);
    x(k+1) = x(k) + dt * u_safe(k);
end

t = (0:N) * dt;
figure;
plot(t, x); hold on;
yline(0, "--");
xlabel("time [s]");
ylabel("x(t)");
title("1D CBF Safety Filter in MATLAB");
