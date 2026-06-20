% Planar 2R parameters
l1 = 1.0;
l2 = 0.7;
omega = 1.5;
T = 5.0;
N = 501;
t = linspace(0, T, N);
h = t(2) - t(1);

q1 = sin(omega * t);
q2 = cos(omega * t);

% joint velocities: numerical (central differences)
dq1_num = zeros(1, N);
dq2_num = zeros(1, N);
dq1_num(1) = (q1(2) - q1(1)) / h;
dq1_num(end) = (q1(end) - q1(end - 1)) / h;
dq2_num(1) = (q2(2) - q2(1)) / h;
dq2_num(end) = (q2(end) - q2(end - 1)) / h;
for k = 2:N-1
    dq1_num(k) = (q1(k + 1) - q1(k - 1)) / (2 * h);
    dq2_num(k) = (q2(k + 1) - q2(k - 1)) / (2 * h);
end

% joint velocities: analytic
dq1_ana = omega * cos(omega * t);
dq2_ana = -omega * sin(omega * t);

% task-space position
x = l1 * cos(q1) + l2 * cos(q1 + q2);
y = l1 * sin(q1) + l2 * sin(q1 + q2);

% task-space velocity: numerical differentiation of x,y
xd_num = zeros(1, N);
yd_num = zeros(1, N);
xd_num(1) = (x(2) - x(1)) / h;
xd_num(end) = (x(end) - x(end - 1)) / h;
yd_num(1) = (y(2) - y(1)) / h;
yd_num(end) = (y(end) - y(end - 1)) / h;
for k = 2:N-1
    xd_num(k) = (x(k + 1) - x(k - 1)) / (2 * h);
    yd_num(k) = (y(k + 1) - y(k - 1)) / (2 * h);
end

% task-space velocity: model-based via Jacobian
xd_model = zeros(1, N);
yd_model = zeros(1, N);
for k = 1:N
    s1 = sin(q1(k));
    c1 = cos(q1(k));
    s12 = sin(q1(k) + q2(k));
    c12 = cos(q1(k) + q2(k));
    J = [ -l1 * s1 - l2 * s12, -l2 * s12;
           l1 * c1 + l2 * c12,  l2 * c12 ];
    dq = [dq1_ana(k); dq2_ana(k)];
    v = J * dq;
    xd_model(k) = v(1);
    yd_model(k) = v(2);
end

% Simple comparison plot
figure;
subplot(2, 1, 1);
plot(t, xd_num, 'b', t, xd_model, 'r--');
legend('xd num', 'xd model');
xlabel('t');
ylabel('xd');

subplot(2, 1, 2);
plot(t, yd_num, 'b', t, yd_model, 'r--');
legend('yd num', 'yd model');
xlabel('t');
ylabel('yd');
      
