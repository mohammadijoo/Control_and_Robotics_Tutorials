% Chapter15_Lesson3.m
% Stiff systems and implicit methods (MATLAB / Simulink-oriented demo)
% 1) Compare ode45 and ode15s on the Robertson problem
% 2) Show recommended Simulink solver settings for stiff models

function Chapter15_Lesson3()
    clc; clear;

    y0 = [1; 0; 0];
    tspan = [0 1e2];

    fprintf('--- Robertson problem: ode45 vs ode15s ---\n');
    opts = odeset('RelTol',1e-6,'AbsTol',1e-10);

    tic;
    [t45, y45] = ode45(@robertson, tspan, y0, opts);
    time45 = toc;

    tic;
    [t15s, y15s] = ode15s(@robertson, tspan, y0, opts);
    time15s = toc;

    fprintf('ode45 : steps = %d, elapsed = %.4f s\n', numel(t45), time45);
    fprintf('ode15s: steps = %d, elapsed = %.4f s\n', numel(t15s), time15s);
    fprintf('Final state ode45 : [%g, %g, %g]\n', y45(end,1), y45(end,2), y45(end,3));
    fprintf('Final state ode15s: [%g, %g, %g]\n', y15s(end,1), y15s(end,2), y15s(end,3));

    figure;
    semilogx(t15s + 1e-12, y15s(:,1), '-', t15s + 1e-12, y15s(:,2), '--', t15s + 1e-12, y15s(:,3), '-.');
    grid on;
    xlabel('t'); ylabel('States');
    title('Robertson problem solved with ode15s');
    legend('y1','y2','y3','Location','best');

    % Simulink note (programmatic setting):
    % If you already have a Simulink model named "StiffPlant", configure solver by:
    % set_param('StiffPlant', 'SolverType', 'Variable-step');
    % set_param('StiffPlant', 'Solver', 'ode15s');
    % set_param('StiffPlant', 'RelTol', '1e-6');
    % set_param('StiffPlant', 'AbsTol', '1e-8');
    % set_param('StiffPlant', 'MaxStep', 'auto');
end

function dydt = robertson(~, y)
    dydt = zeros(3,1);
    dydt(1) = -0.04*y(1) + 1e4*y(2)*y(3);
    dydt(2) =  0.04*y(1) - 1e4*y(2)*y(3) - 3e7*y(2)^2;
    dydt(3) =  3e7*y(2)^2;
end
