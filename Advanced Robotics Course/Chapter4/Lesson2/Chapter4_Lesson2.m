function q = chomp_2d_step(q, K, alpha, lambda)
% q: N x 2 internal waypoints
% K: N x N smoothness matrix

N = size(q, 1);
q_flat = q';
q_flat = q_flat(:);

Kbig = kron(K, eye(2));
grad_smooth = Kbig * q_flat;

grad_obs = zeros(size(q));
for k = 1:N
    grad_obs(k, :) = obstacle_grad(q(k, :));
end
grad_obs_flat = grad_obs';
grad_obs_flat = grad_obs_flat(:);

grad_total = grad_smooth + lambda * grad_obs_flat;

A = Kbig + 1e-6 * eye(2 * N);
delta = A \ grad_total;

q_new_flat = q_flat - alpha * delta;
q_new = reshape(q_new_flat, 2, N)';
q = q_new;
end

function c = obstacle_cost(x)
d0 = 0.5;
d = norm(x);
if d < d0
    diff = d0 - d;
    c = 0.5 * diff^2;
else
    c = 0.0;
end
end

function g = obstacle_grad(x)
d0 = 0.5;
d = norm(x);
if d < 1e-8
    g = [0.0, 0.0];
elseif d < d0
    coeff = -(d0 - d) / d;
    g = coeff * x;
else
    g = [0.0, 0.0];
end
end
      
