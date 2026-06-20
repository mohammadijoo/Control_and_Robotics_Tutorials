
m = 1.0;
k_env = 5000.0;
x0 = 0.05;

m_c = 0.5;
k_c = 2000.0;
d_c = 2.0 * sqrt(m_c * k_c);

f_n_des = 20.0;
x_d = 0.0;

T_s = 5e-4;
T_end = 0.8;
N = floor(T_end / T_s);

x = zeros(1, N);
v = zeros(1, N);
f_env = zeros(1, N);
f_cmd = zeros(1, N);
e = zeros(1, N);

x(1) = x0;
v(1) = 0.0;
e_prev1 = 0.0;
e_prev2 = 0.0;

for k = 3:N
    if x(k-1) > 0.0
        f_env(k-1) = 0.0;
    else
        f_env(k-1) = -k_env * x(k-1);
    end

    e(k) = x(k-1) - x_d;

    de = (e(k) - e_prev1) / T_s;
    dde = (e(k) - 2.0 * e_prev1 + e_prev2) / (T_s^2);

    f_c = m_c * dde + d_c * de + k_c * e(k) - f_env(k-1) + f_n_des;
    f_c = max(min(f_c, 200.0), -200.0);

    f_cmd(k) = f_c;

    a = (f_c + f_env(k-1)) / m;
    v(k) = v(k-1) + T_s * a;
    x(k) = x(k-1) + T_s * v(k);

    e_prev2 = e_prev1;
    e_prev1 = e(k);
end

t = (0:N-1) * T_s;
figure;
subplot(2,1,1);
plot(t, x); hold on; yline(0.0, "--");
ylabel("x (m)");
subplot(2,1,2);
plot(t, f_env); hold on; yline(f_n_des, "--");
ylabel("Force (N)"); xlabel("Time (s)");
