% Physical and control parameters
J = 0.5;
B = 0.1;
omega_n = 4.0;
zeta = 0.7;

Kp = J * omega_n^2;
Kd = 2*zeta*omega_n*J - B;
tau_max = 10;

% Continuous-time closed-loop error dynamics:
% J*e_ddot + (B + Kd)*e_dot + Kp*e = 0
A_e = [0 1; -Kp/J -(B+Kd)/J];
B_e = [0; 0];
C_e = [1 0];
D_e = 0;
sys_e = ss(A_e,B_e,C_e,D_e);

% Example: step in reference
t = 0:0.001:5;
u = zeros(size(t));  % error system has zero input
[y, t_out] = lsim(sys_e, u, t, [0; 0]); % free response

figure;
plot(t_out, y);
xlabel('Time (s)');
ylabel('Tracking error e(t)');
title('Error dynamics for PD-controlled assistive joint');

% In Simulink, one would typically use:
% - a "Sum" block to compute e = theta_d - theta
% - a "Derivative" or filtered differentiator for e_dot
% - "Gain" blocks for Kp and Kd
% - a "Saturation" block with limits [-tau_max, tau_max]
% - a plant block J*theta_ddot + B*theta_dot = tau_total
      
