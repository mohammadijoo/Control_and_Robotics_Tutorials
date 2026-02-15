dt = 0.05;
A = [1 dt; 0 1];
B = [0.5 * dt^2; dt];
K = [-2 -1];

p_max = 5.0;
v_max = 3.0;
T = 200;
N_trials = 5000;
sigma_w = 0.05;

failures = 0;
for i = 1:N_trials
    % Random initial state
    x = [ -4 + 8*rand();
          -2 + 4*rand() ];
    failed = false;
    for k = 1:T
        u = K * x;
        w = sigma_w * randn(2,1);
        x = A * x + B * u + w;
        if abs(x(1)) > p_max || abs(x(2)) > v_max
            failed = true;
            break;
        end
    end
    if failed
        failures = failures + 1;
    end
end

p_hat = failures / N_trials;
fprintf('Empirical failure rate: %.4f\n', p_hat);

% Simulink hint:
% The same dynamics can be implemented using a "State-Space" block with (A,B,[],[])
% plus a gain block for K and a noise source feeding into the state update.
      
