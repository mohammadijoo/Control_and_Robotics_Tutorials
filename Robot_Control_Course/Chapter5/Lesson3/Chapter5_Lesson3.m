
function tau = saturated_pd(q, qd, q_ref, qd_ref, Kp, Kd, tau_min, tau_max)
% q, qd, q_ref, qd_ref: n-by-1 vectors
% Kp, Kd, tau_min, tau_max: n-by-1 vectors

e  = q  - q_ref;
ed = qd - qd_ref;

tau_nom = -Kp .* e - Kd .* ed;
tau     = min(max(tau_nom, tau_min), tau_max);
