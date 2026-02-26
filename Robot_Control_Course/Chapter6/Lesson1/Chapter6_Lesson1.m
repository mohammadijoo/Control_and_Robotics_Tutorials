
m   = 1.0;
b   = 1.0;
k_p = 200.0;
k_d = 20.0;
k_e = 5e3;
x_s = 0.0;
F_max = 80.0;

dt = 5e-4;
T  = 1.0;
N  = round(T / dt);

x = -0.02;
v = 0.0;

xs = zeros(N,1);
Fs = zeros(N,1);
ts = (0:N-1)' * dt;

for i = 1:N
    t = ts(i);
    if t < 0.1
        xd = -0.02;
    else
        xd = 0.03;
    end

    if x <= x_s
        F_e = 0;
    else
        F_e = k_e * (x - x_s);
    end

    e = xd - x;
    u = k_p * e - k_d * v;

    if F_e > F_max
        u = u - (F_e - F_max);
    end

    a = (u - F_e - b * v) / m;
    v = v + dt * a;
    x = x + dt * v;

    xs(i) = x;
    Fs(i) = F_e;
end

% Plot results
figure; subplot(2,1,1);
plot(ts, xs); ylabel('x (m)'); grid on;
subplot(2,1,2);
plot(ts, Fs); ylabel('F_e (N)'); xlabel('t (s)'); grid on;
