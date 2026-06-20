% Residual-based detector for discrete signals
S = eye(2);
Sinv = inv(S);
gamma = 9.21; % chi^2_2 95% threshold

for k = 1:50
    y = randn(2,1);
    y_hat = zeros(2,1);
    r = y - y_hat;
    g = r' * Sinv * r;
    if g > gamma
        fprintf('[FAULT] k=%d, score=%.2f\n', k, g);
        % In Simulink, trigger safe-mode via Stateflow transition
    end
end
      
