% Chapter26_Lesson5.m
% Servo design by state augmentation using MATLAB Control System Toolbox.
% The final section creates a small Simulink-style closed-loop model skeleton.

clear; clc; close all;

% Plant: mass-spring-damper position servo
A = [0 1; -2 -0.8];
B = [0; 1];
C = [1 0];
D = 0;

n = size(A,1);
p = size(C,1);
m = size(B,2);

% Integral state: eta_dot = r - y = r - Cx for D = 0.
Aa = [A zeros(n,p); -C zeros(p,p)];
Ba = [B; zeros(p,m)];
Ea = [zeros(n,p); eye(p)];

Wc = ctrb(Aa, Ba);
fprintf('Augmented controllability rank = %d of %d\n', rank(Wc), n+p);
if rank(Wc) < n+p
    error('The augmented system is not controllable.');
end

poles = [-2 -2.5 -3];
Kaug = place(Aa, Ba, poles);
Kx = Kaug(:,1:n);
Ki = -Kaug(:,n+1:end);

Acl = Aa - Ba*Kaug;
Cr = [C zeros(p,p)];
Dr = zeros(p,p);

fprintf('Kx = '); disp(Kx);
fprintf('Ki = '); disp(Ki);
fprintf('Closed-loop poles = '); disp(eig(Acl).');

sys_cl = ss(Acl, Ea, Cr, Dr);
t = 0:0.01:8;
r = ones(size(t));
y = lsim(sys_cl, r, t);

z = lsim(ss(Acl, Ea, eye(n+p), zeros(n+p,p)), r, t);
u = zeros(length(t),1);
for k = 1:length(t)
    xk = z(k,1:n).';
    etak = z(k,n+1:end).';
    u(k) = -Kx*xk + Ki*etak;
end

figure; plot(t, y, 'LineWidth', 1.5); hold on; plot(t, r, '--'); grid on;
xlabel('Time [s]'); ylabel('Output'); title('State-Augmented Servo Response');
legend('y(t)', 'r');

figure; plot(t, u, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('Control input'); title('Servo Control Effort');

% Minimal Simulink-style model skeleton.
% This block-level model uses the already designed closed-loop matrix Acl.
% For a full implementation, replace the closed-loop State-Space block by
% separate Plant, Integrator, Sum, and Gain blocks.
try
    model = 'Chapter26_Lesson5_SimulinkServo';
    if bdIsLoaded(model), close_system(model,0); end
    new_system(model); open_system(model);

    add_block('simulink/Sources/Step', [model '/Reference Step'], ...
        'Position', [40 80 90 110], 'Time', '0', 'Before', '0', 'After', '1');
    add_block('simulink/Continuous/State-Space', [model '/Closed Loop Augmented Servo'], ...
        'Position', [150 65 320 125], ...
        'A', 'Acl', 'B', 'Ea', 'C', 'Cr', 'D', 'Dr');
    add_block('simulink/Sinks/Scope', [model '/Output Scope'], ...
        'Position', [380 75 430 115]);
    add_line(model, 'Reference Step/1', 'Closed Loop Augmented Servo/1');
    add_line(model, 'Closed Loop Augmented Servo/1', 'Output Scope/1');
    set_param(model, 'StopTime', '8');
    save_system(model);
    fprintf('Created Simulink model skeleton: %s.slx\n', model);
catch ME
    warning('Simulink model skeleton was not created: %s', ME.message);
end
