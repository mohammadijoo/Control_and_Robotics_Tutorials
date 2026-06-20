function q_new = tactileGraspRefineStep(q, contacts, Jc_fn, Jn_fn, ...
    w_slip, w_pos, w_align, step_size, q_min, q_max)
%TACTILEGRASPREFINESTEP One gradient-based refinement step.
%
%   q:        n x 1 joint vector
%   contacts: struct array with fields positions (Kx3),
%             pressures (Kx1), areas (Kx1),
%             c_des (1x3), n_des (1x3), mu (scalar)
%   Jc_fn:    function handle @(q,j) -> 3 x n contact Jacobian
%   Jn_fn:    function handle @(q,j) -> 3 x n normal Jacobian

if nargin < 5 || isempty(w_slip),  w_slip  = 1.0;  end
if nargin < 6 || isempty(w_pos),   w_pos   = 1.0;  end
if nargin < 7 || isempty(w_align), w_align = 0.1;  end
if nargin < 8 || isempty(step_size), step_size = 1e-2; end

n = numel(q);
grad = zeros(n,1);

for j = 1:numel(contacts)
    c = contacts(j);
    w = c.pressures(:) .* c.areas(:);
    F_n = sum(w) + 1e-9;
    num = c.positions.' * w;
    c_est = num / F_n;

    pos_err = c_est(:) - c.c_des(:);
    f_t_norm_est = norm(pos_err);
    h = f_t_norm_est / (c.mu * F_n + 1e-9) - 1.0;
    slip_penalty = max(0.0, h);

    Jc = Jc_fn(q, j); % 3 x n
    Jn = Jn_fn(q, j); % 3 x n

    grad_pos = 2.0 * Jc.' * pos_err;

    n_est = zeros(3,1); % placeholder
    n_err = n_est - c.n_des(:);
    grad_align = 2.0 * Jn.' * n_err;

    grad_slip = zeros(n,1);
    if slip_penalty > 0.0 && f_t_norm_est > 1e-6
        dir_vec = pos_err / f_t_norm_est;
        dh_dq = (Jc.' * dir_vec) / (c.mu * F_n + 1e-9);
        grad_slip = 2.0 * slip_penalty * dh_dq;
    end

    grad = grad + w_pos * grad_pos + ...
                 w_align * grad_align + ...
                 w_slip * grad_slip;
end

dq = -step_size * grad;
q_new = q(:) + dq;

if nargin >= 9 && ~isempty(q_min)
    q_new = max(q_new, q_min(:));
end
if nargin >= 10 && ~isempty(q_max)
    q_new = min(q_new, q_max(:));
end
      
