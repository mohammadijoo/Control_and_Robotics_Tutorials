% Chapter27_Lesson3.m
% Internal Model Principle in a linear state-space setting.
% Requires Control System Toolbox for place().
% Optional: creates a simple Simulink model if Simulink is available.

clear; clc; close all;

A = [0 1; -2 -0.4];
B = [0; 1];
C = [1 0];
D = 0;

% Step-reference internal model: eta_dot = r - y.
Aaug = [A zeros(2,1); -C 0];
Baug = [B; 0];

desired_poles = [-2+2i, -2-2i, -5];
Kaug = place(Aaug, Baug, desired_poles);

Kx = Kaug(1:2);
Ki = -Kaug(3);

fprintf('Kx = [%g  %g]\n', Kx(1), Kx(2));
fprintf('Ki = %g\n', Ki);

r = 1.0;
f = @(t,xa) [
    xa(2);
    -2*xa(1) - 0.4*xa(2) + (-Kx*[xa(1); xa(2)] + Ki*xa(3));
    r - C*[xa(1); xa(2)]
];

[t, xa] = ode45(f, [0 8], [0; 0; 0]);
y = xa(:,1);
u = -Kx(1)*xa(:,1) - Kx(2)*xa(:,2) + Ki*xa(:,3);
e = r - y;

figure;
plot(t, y, 'LineWidth', 1.5); grid on;
xlabel('Time (s)');
ylabel('Output y(t)');
title('Step Tracking with an Internal Model');

figure;
plot(t, e, 'LineWidth', 1.5); grid on;
xlabel('Time (s)');
ylabel('Tracking error e(t)');
title('Error Convergence');

fprintf('Final y(T) = %.8f\n', y(end));
fprintf('Final e(T) = %.8e\n', e(end));
fprintf('Peak |u| = %.8f\n', max(abs(u)));

% Regulator equations for constant reference:
% A*Pi + B*Gamma = 0, C*Pi = 1.
M = [A B; C 0];
rhs = [0; 0; 1];
sol = M \ rhs;
Pi = sol(1:2);
Gamma = sol(3);
fprintf('Pi = [%g  %g]^T\n', Pi(1), Pi(2));
fprintf('Gamma = %g\n', Gamma);

% Optional Simulink skeleton: integrator state eta and State-Space plant block.
if license('test','SIMULINK')
    model = 'Chapter27_Lesson3_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Step', [model '/Reference'], ...
        'Position', [40 80 80 110], 'Time', '0', 'Before', '0', 'After', '1');
    add_block('simulink/Continuous/State-Space', [model '/Plant'], ...
        'Position', [320 70 430 130], ...
        'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));
    add_block('simulink/Math Operations/Sum', [model '/Error r-y'], ...
        'Position', [130 75 160 115], 'Inputs', '+-');
    add_block('simulink/Continuous/Integrator', [model '/Internal Model Integrator'], ...
        'Position', [190 75 230 115]);
    add_block('simulink/Sinks/Scope', [model '/Output Scope'], ...
        'Position', [500 75 540 115]);

    % Full feedback interconnection usually needs gain and sum blocks for
    % u = -Kx + Ki*eta. This skeleton shows the internal-model signal path.
    add_line(model, 'Reference/1', 'Error r-y/1');
    add_line(model, 'Error r-y/1', 'Internal Model Integrator/1');
    add_line(model, 'Plant/1', 'Output Scope/1');

    save_system(model);
    fprintf('Created Simulink skeleton: %s.slx\n', model);
end
