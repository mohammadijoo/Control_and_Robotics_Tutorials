function model = trainAffordanceLogistic(Phi, y, lr, reg, nEpoch)
% Phi: N-by-d feature matrix
% y:   N-by-1 labels in {0,1}
% lr:  learning rate
% reg: L2 regularization
% nEpoch: number of passes over data

[N, d] = size(Phi);
theta = zeros(d, 1);

for epoch = 1:nEpoch
    % Shuffle indices
    idx = randperm(N);
    Phi_shuf = Phi(idx, :);
    y_shuf = y(idx);

    for i = 1:N
        phi_i = Phi_shuf(i, :).';
        z = theta.' * phi_i;
        p = 1.0 / (1.0 + exp(-z));

        % Gradient of negative log-likelihood with L2
        grad = -(y_shuf(i) - p) * phi_i + reg * theta;
        theta = theta - lr * grad;
    end
end

model.theta = theta;
end

function p = predictAffordance(model, Phi_query)
% Phi_query: M-by-d feature matrix
theta = model.theta;
z = Phi_query * theta;
p = 1.0 ./ (1.0 + exp(-z));
end
      
