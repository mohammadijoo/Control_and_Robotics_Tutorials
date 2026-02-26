% Parameters
alpha_values = [0.2, 0.9];
k = 2;

s = tf('s');

for idx = 1:numel(alpha_values)
    alpha = alpha_values(idx);

    % 2x2 coupled plant: G(s) = 1/(s+1)*[1 alpha; alpha 1]
    G11 = 1/(s + 1);
    G22 = 1/(s + 1);
    G12 = alpha/(s + 1);
    G21 = alpha/(s + 1);
    G = [G11 G12; G21 G22];

    % Decentralized proportional controller
    C = k * eye(2);

    % Loop transfer and closed-loop from r to y
    L = G * C;
    I2 = eye(2);
    T = feedback(L, I2);  % T(s) = L(I+L)^(-1)

    % Step in r1 only
    t = 0:0.01:10;
    r = [ones(size(t)); zeros(size(t))];
    [y, t_out] = lsim(T, r', t);

    figure;
    plot(t_out, y(:,1), 'b', 'LineWidth', 1.5); hold on;
    plot(t_out, y(:,2), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Outputs');
    title(sprintf('Step in r1, alpha = %.2f', alpha));
    legend('y1', 'y2');
end
