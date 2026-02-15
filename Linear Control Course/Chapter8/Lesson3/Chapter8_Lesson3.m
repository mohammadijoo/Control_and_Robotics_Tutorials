% Open-loop transfer function
s = tf('s');
G = 10 / (s * (0.5*s + 1));
C = 1;
L = C*G;

% Error transfer E(s)/R(s) for unity feedback
E_over_R = feedback(1, L);

% Static error constants
Kp = dcgain(L);
Kv = dcgain(s*L);
Ka = dcgain(s^2*L);

fprintf('Kp = %g\n', Kp);
fprintf('Kv = %g\n', Kv);
fprintf('Ka = %g\n', Ka);

% Numerical steady-state errors
[et, t_step] = step(E_over_R);
e_step_ss = et(end);

t = linspace(0, 50, 2000);
r_ramp = t;           % unit ramp
[er_ramp, t_ramp] = lsim(E_over_R, r_ramp, t);
e_ramp_ss = er_ramp(end);

r_par = 0.5*t.^2;    % unit parabolic
[er_par, t_par] = lsim(E_over_R, r_par, t);
e_par_ss = er_par(end);

fprintf('Step e_ss (sim)      = %g\n', e_step_ss);
fprintf('Ramp e_ss (sim)      = %g\n', e_ramp_ss);
fprintf('Parabolic e_ss (sim) = %g\n', e_par_ss);

% Theoretical values
fprintf('Step e_ss (theory)   = %g\n', 1/(1 + Kp));
fprintf('Ramp e_ss (theory)   = %g\n', 1/Kv);
