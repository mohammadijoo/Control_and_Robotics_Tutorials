
function metrics = compute_metrics(t, q, qd, tau)
% t: Nx1 time vector
% q, qd, tau: Nxnjoint arrays

e = q - qd;
e2 = sum(e.^2, 2);
u2 = sum(tau.^2, 2);

J_ISE = trapz(t, e2);
J_ISU = trapz(t, u2);

% overshoot and settling time for first joint
e1 = e(:,1);
r_step = 1.0;
Mp = max(abs(e1)) / abs(r_step) * 100;

eps = 0.02;
band = eps * abs(r_step);
settled_idx = find(all(abs(e1(end:-1:1)) <= band), 1, "last");
if isempty(settled_idx)
    t_s = inf;
else
    t_s = t(settled_idx);
end

metrics.J_ISE = J_ISE;
metrics.J_ISU = J_ISU;
metrics.Mp = Mp;
metrics.t_s = t_s;
