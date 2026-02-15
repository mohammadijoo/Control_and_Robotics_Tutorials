
A = [0 1; 0 -2];
B = [0; 1];

h_nom = 0.002;
jitter_max = 0.0005;

Kp = 50;
Kd = 5;

x = [0.5; 0.0];
xref = 0.0;

T = 2.0;          % total simulated time
t = 0.0;
k = 0;
log_t = [];
log_x = [];

while t < T
    pos = x(1);
    vel = x(2);
    e = xref - pos;
    u = Kp * e - Kd * vel;

    h = h_nom + (2*rand - 1)*jitter_max;
    xdot = A * x + B * u;
    x = x + h * xdot;

    t = t + h;
    k = k + 1;
    log_t(k,1) = t;
    log_x(k,1:2) = x.';
end

plot(log_t, log_x(:,1));
xlabel('time (s)'); ylabel('position');
title('Joint position under sampling jitter');
