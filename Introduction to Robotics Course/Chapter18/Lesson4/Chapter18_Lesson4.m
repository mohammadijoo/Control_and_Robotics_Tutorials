% Parameters
dt = 0.001;
T  = 5.0;
steps = floor(T/dt);

a_true = 2.0;
K      = 10.0;
gamma  = 5.0;

x     = 0.0;
a_hat = 0.0;

x_hist = zeros(steps,1);
a_hist = zeros(steps,1);

for k = 1:steps
    t  = (k-1)*dt;
    if t > 0.5
        x_ref = 1.0;
    else
        x_ref = 0.0;
    end

    e   = x - x_ref;
    u_fb = -K * e;
    u_ff = a_hat * x;
    u    = u_fb + u_ff;

    x_dot = u - a_true * x;
    x = x + dt * x_dot;

    a_hat = a_hat - gamma * e * x * dt;

    x_hist(k) = x;
    a_hist(k) = a_hat;
end

% Plot (optional)
figure; plot((0:steps-1)*dt, x_hist, (0:steps-1)*dt, x_hist*0 + 1.0, '--');
xlabel('time [s]'); ylabel('velocity'); legend('x','x\_ref');

figure; plot((0:steps-1)*dt, a_hist); hold on;
yline(a_true,'--','a\_true');
xlabel('time [s]'); ylabel('a\_hat');
legend('a\_hat','a\_true');
      
