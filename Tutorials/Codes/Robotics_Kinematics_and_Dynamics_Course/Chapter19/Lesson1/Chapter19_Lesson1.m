function tau = one_link_torque(q, qd, qdd, theta)
% theta = [I; b; fc; mgL]

Y = [qdd, qd, sign(qd), sin(q)];
tau = Y * theta;
end

% Example usage:
theta_true = [0.8; 0.05; 0.2; 3.0];
theta_nom  = [1.0; 0.02; 0.1; 2.5];

q   = 0.5;
qd  = 0.1;
qdd = -0.3;

tau_true = one_link_torque(q, qd, qdd, theta_true);
tau_nom  = one_link_torque(q, qd, qdd, theta_nom);
disp(['tau_true = ', num2str(tau_true)])
disp(['tau_nom  = ', num2str(tau_nom)])
disp(['difference = ', num2str(tau_nom - tau_true)])

% In Simulink, one can implement Y(q, qd, qdd) as a subsystem
% and multiply by the parameter vector theta (e.g., using a Gain
% block or Matrix Multiply block) to obtain tau.
      
