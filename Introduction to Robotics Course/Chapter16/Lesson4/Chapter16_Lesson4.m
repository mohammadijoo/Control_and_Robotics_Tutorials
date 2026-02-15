K  = 1.0;
tau = 0.5;

% Plant and proportional controller
s = tf('s');
G = K / (tau * s + 1);
Kp_values = [0.5 1.0 2.0 4.0];

for Kp = Kp_values
    C = Kp;
    T = feedback(C * G, 1);  % closed-loop transfer

    % Step response and metrics
    [y, t] = step(T, 5);
    y_inf = y(end);
    e_inf = 1 - y_inf;
    Mp = 100 * (max(y) - y_inf) / y_inf;

    idx = find(abs(y - y_inf) > 0.02 * abs(y_inf), 1, 'last');
    if isempty(idx)
        Ts = 0;
    else
        Ts = t(idx);
    end

    fprintf('Kp=%.2f, e_inf=%.3f, Mp=%.1f, Ts=%.2f\n', Kp, e_inf, Mp, Ts);
end

% Simulink notes:
% - Create a model with:
%   Step block --> Gain (Kp) --> Transfer Fcn (K / (tau s + 1)) --> Scope.
% - Use Simulation -> Model Explorer to sweep Kp and log response data.
      
