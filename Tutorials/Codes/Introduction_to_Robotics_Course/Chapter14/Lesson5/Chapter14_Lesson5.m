a = 0.9;
b = 0.3;
K = 0.8;
x_ref = 0.7;

N = 40;
x = zeros(1, N+1);
u = zeros(1, N);
x(1) = 0.1;

for k = 1:N
    u(k) = -K * (x(k) - x_ref);
    x(k+1) = a * x(k) + b * u(k);
end

k = 0:N;
figure;
subplot(2,1,1);
stairs(k, x, "LineWidth", 1.5);
xlabel("k");
ylabel("engagement x_k");
grid on;

subplot(2,1,2);
stairs(0:N-1, u, "LineWidth", 1.5);
xlabel("k");
ylabel("control u_k");
grid on;
      
