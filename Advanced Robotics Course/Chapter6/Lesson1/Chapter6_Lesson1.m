dt = 0.1;

f = @(x, u) [ ...
    x(1) + u(1)*cos(x(3))*dt; ...
    x(2) + u(1)*sin(x(3))*dt; ...
    x(3) + u(2)*dt];

Q = diag([0.01^2, 0.01^2, deg2rad(1)^2]);

N = 5000;
x0 = [0; 0; 0];
u = [1; 0.2];

X = zeros(3, N);
for k = 1:N
    x = x0;
    for t = 1:20
        x_nom = f(x, u);
        w = mvnrnd([0 0 0], Q).';
        x = x_nom + w;
    end
    X(:,k) = x;
end

mu_hat = mean(X, 2);
Sigma_hat = cov(X.');

disp('Estimated mean:');
disp(mu_hat);
disp('Estimated covariance:');
disp(Sigma_hat);

% Simulink suggestion:
% Build a model where:
%  - A MATLAB Function block implements f(x,u).
%  - Add Random Number blocks for each state component, scaled by sqrt of Q.
%  - Sum them into the state update.
%  - Use Scope blocks to visualize stochastic trajectories.
      
