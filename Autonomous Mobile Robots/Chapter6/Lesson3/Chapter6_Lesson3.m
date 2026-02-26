% Chapter6_Lesson3.m
% Autonomous Mobile Robots (Control Engineering) - Chapter 6, Lesson 3
% Motion Models vs Sensor Models: A discrete Bayes-filter demonstration (1D grid)

clear; clc;

% World definition
N = 121;
dx = 0.1;
beacon_x = 6.0;
xs = (0:N-1) * dx;

% Initial belief (uniform)
bel = ones(N,1);
bel = bel / sum(bel);

% Sequence of controls and measurements
u_seq = [0.5, 0.5, 0.5, 0.5];          % meters
z_seq = [5.5, 5.0, 4.5, 4.0];          % measured distance to beacon

sigma_u = 0.25;  % motion noise
sigma_z = 0.35;  % sensor noise

fprintf('t |  MAP estimate (m) |  belief entropy (nats)\n');
fprintf('--+-------------------+----------------------\n');

for t = 1:length(u_seq)
    u = u_seq(t);
    z = z_seq(t);

    % Motion prediction: bel_bar(x) = sum_x' N(x; x'+u, sigma_u^2) bel(x')
    bel_bar = zeros(N,1);
    for i = 1:N
        mu = xs(i) + u;
        kernel = (1/(sqrt(2*pi)*sigma_u)) * exp(-0.5*((xs - mu)/sigma_u).^2)';
        bel_bar = bel_bar + bel(i) * kernel;
    end
    bel_bar = bel_bar / sum(bel_bar);

    % Sensor update: p(z|x) = N(z; |x - beacon|, sigma_z^2)
    expected = abs(xs - beacon_x);
    likelihood = (1/(sqrt(2*pi)*sigma_z)) * exp(-0.5*((z - expected)/sigma_z).^2)';
    bel = bel_bar .* likelihood;
    bel = bel / sum(bel);

    [~, idx] = max(bel);
    x_map = xs(idx);
    entropy = -sum(bel(bel>0) .* log(bel(bel>0)));

    fprintf('%d | %17.3f | %20.6f\n', t, x_map, entropy);
end

% Save final belief
T = table(xs', bel, 'VariableNames', {'x','bel'});
writetable(T, 'Chapter6_Lesson3_belief_final.csv');
fprintf('\nSaved: Chapter6_Lesson3_belief_final.csv\n');
