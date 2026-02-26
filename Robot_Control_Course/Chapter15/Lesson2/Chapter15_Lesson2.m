
function tau_cmd = residual_controller(q, dq, qd, dqd, ddqd, params)
% params.W : 1 x m row vector
% params.b : scalar
% Build features z = [q; dq; qd; dqd; ddqd]
z = [q; dq; qd; dqd; ddqd];

% Nominal torque from rigid-body model
tau_nom = nominal_torque(q, dq, ddqd);

% Residual torque
tau_res = params.W * z + params.b;

% Command torque
tau_cmd = tau_nom + tau_res;
end
