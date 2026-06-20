% Second-order plant parameters
zeta = 0.3;
omega_n = 4.0;
k_plant = 1.0;

Kp1 = 20.0;
Ky2 = 20.0;
Kr2 = 20.0;
Tf = 0.2;

dt = 1e-3;
T_end = 2.0;
steps = round(T_end / dt);

x1_pos = 0.0; x1_vel = 0.0;
x2_pos = 0.0; x2_vel = 0.0;
r = 1.0;
rf = 0.0;

y1 = zeros(steps,1);
y2 = zeros(steps,1);
t = (0:steps-1)' * dt;

for kStep = 1:steps
    y1(kStep) = x1_pos;
    y2(kStep) = x2_pos;

    u1 = Kp1 * (r - x1_pos);

    rf = rf + dt * (r - rf) / Tf;
    u2 = Kr2 * rf - Ky2 * x2_pos;

    dx1_pos = x1_vel;
    dx1_vel = -2*zeta*omega_n*x1_vel ...
              - omega_n^2 * x1_pos + k_plant * u1;
    x1_pos = x1_pos + dt * dx1_pos;
    x1_vel = x1_vel + dt * dx1_vel;

    dx2_pos = x2_vel;
    dx2_vel = -2*zeta*omega_n*x2_vel ...
              - omega_n^2 * x2_pos + k_plant * u2;
    x2_pos = x2_pos + dt * dx2_pos;
    x2_vel = x2_vel + dt * dx2_vel;
end

plot(t, y1, t, y2);
legend('1-DOF','2-DOF');
xlabel('Time (s)');
ylabel('Position');
title('Step response: 1-DOF vs 2-DOF');

% Transfer function representation (Control System Toolbox)
s = tf('s');
G = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2);
Cy = Kp1;
F = 1 / (Tf*s + 1);
Cr = Kp1 * F;

T_1dof = feedback(G*Cy, 1);        % Y/R for 1-DOF
T_2dof = (G*Cr) / (1 + G*Cy);      % Y/R for 2-DOF (explicit formula)
