% Parameters
N   = 20;
d   = 2;
a   = 0.5;
b   = 1.0;
eps = 1e-2;
R   = 1.0;
h   = 0.02;
steps = 1000;

% Initialize positions
rng(0);
x = -2 + 4 * rand(N, d);

phi_prime = @(r) 2*a*r - b./(r + eps);

for k = 1:steps
    x_new = x;
    for i = 1:N
        force = zeros(1, d);
        for j = 1:N
            if i == j, continue; end
            diff = x(i,:) - x(j,:);
            dist = norm(diff);
            if dist < 1e-6 || dist > R, continue; end
            fmag = phi_prime(dist);
            force = force + (fmag / dist) * diff;
        end
        x_new(i,:) = x(i,:) - h * force;
    end
    x = x_new;
end

scatter(x(:,1), x(:,2), 'filled');
axis equal;
title('Final swarm configuration');
      
