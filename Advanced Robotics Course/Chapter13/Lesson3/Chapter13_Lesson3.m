function [theta_best, J_best] = calibrate_simulink(realData, modelName)
% realData: struct array with fields u, y_real, t
% modelName: name of Simulink model, parameterized by base workspace 'theta'

    theta0 = [1.0; 1.0; 1.0];  % initial guess
    lb = [0.5; 0.5; 0.5];
    ub = [1.5; 1.5; 1.5];

    cost_fun = @(theta) calibrationObjective(theta, realData, modelName);

    opts = optimoptions('patternsearch', ...
        'Display', 'iter', ...
        'MaxIterations', 50);

    [theta_best, J_best] = patternsearch(cost_fun, theta0, ...
        [], [], [], [], lb, ub, [], opts);
end

function J = calibrationObjective(theta, realData, modelName)
    assignin('base', 'theta', theta);  % push to base workspace

    J = 0.0;
    for i = 1:numel(realData)
        exp = realData(i);
        in = Simulink.SimulationInput(modelName);
        in = in.setExternalInput([exp.t, exp.u]);
        out = sim(in, 'ShowProgress', 'off');

        y_sim = out.logsout.getElement('y').Values.Data;
        y_real = exp.y_real;

        % simple quadratic feature mismatch on joint angles
        diff = y_real - y_sim;
        J = J + sum(diff(:).^2);
    end
end
      
