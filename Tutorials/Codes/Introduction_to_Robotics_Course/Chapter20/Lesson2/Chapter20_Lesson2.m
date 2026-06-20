% SDR specifications
Mp_max = 0.10;
Ts_max = 1.5;

% Candidate second-order closed-loop transfer function
zeta = 0.7;
omega_n = 4.0;
num = [omega_n^2];
den = [1, 2*zeta*omega_n, omega_n^2];
Gcl = tf(num, den);

info = stepinfo(Gcl);
Mp = info.Overshoot / 100.0;
Ts = info.SettlingTime;

fprintf("Overshoot: %.3f, Ts: %.3f s\n", Mp, Ts);

if Mp <= Mp_max && Ts <= Ts_max
    disp("SDR check PASSED: performance requirements satisfied.");
else
    disp("SDR check FAILED: adjust controller gains.");
end

% In Simulink, connect the plant and controller blocks,
% run a step response simulation, and call stepinfo on
% the logged output signal for automated SDR checks.
      
