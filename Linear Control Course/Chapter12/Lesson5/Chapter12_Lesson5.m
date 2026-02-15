% Motor parameters (normalized)
K = 10; tau_m = 0.05;
s = tf('s');
P = K / (s*(tau_m*s + 1));

% PID gains (obtained from previous design steps)
Kp = 2.0; Ki = 50.0; Kd = 0.001;
Ts = 0.001; alpha = 0.9; Kaw = 20;

u_min = -12; u_max = 12;

T = 0.5;
N = round(T/Ts);

theta = zeros(1,N+1);
theta_dot = zeros(1,N+1);
u = zeros(1,N+1);
r = ones(1,N+1); % 1 rad step

I = 0; D = 0; e_prev = 0;

for k = 1:N
    e = r(k) - theta(k);

    % Derivative filter
    D = alpha*D + (1-alpha)*(e - e_prev)/Ts;

    v = Kp*e + I + Kd*D;
    u(k) = min(max(v, u_min), u_max);

    % Anti-windup
    I = I + Ki*Ts*e + Kaw*(u(k) - v);

    % Motor dynamics: theta_ddot = -(1/tau_m)*theta_dot + (K/tau_m)*u
    theta_ddot = -(1/tau_m)*theta_dot(k) + (K/tau_m)*u(k);

    theta_dot(k+1) = theta_dot(k) + Ts*theta_ddot;
    theta(k+1) = theta(k) + Ts*theta_dot(k+1);

    e_prev = e;
end

% Plot step response
t = (0:N)*Ts;
figure;
subplot(2,1,1); plot(t, r, '--', t, theta); grid on;
ylabel('theta (rad)');
subplot(2,1,2); plot(t, u); grid on;
ylabel('u (V)'); xlabel('time (s)');

% In Simulink, one can instead use:
% - A PID Controller block in discrete-time mode
% - Saturation block for actuator limits
% - Anti-windup option in the PID block (back-calculation)
% connected to a motor plant block or Transfer Fcn block P.
