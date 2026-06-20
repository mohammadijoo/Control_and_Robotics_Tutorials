% Chapter28_Lesson2.m
% Energy-like measures and norms of signals for a state-space model.
% MATLAB libraries/toolboxes: Control System Toolbox, Robust Control Toolbox,
% Optimization Toolbox, and Simulink for block-diagram simulation.

clear; clc;

A = [0 1; -4 -0.8];
B = [0; 1];
C = eye(2);
D = [0; 0];
x0 = [1; 0];
Qx = diag([10 1]);
Ru = 0.2;
Wy = diag([1 0.1]);

u = @(t) 0.5*sin(3*t).*exp(-0.2*t);
f = @(t,x) A*x + B*u(t);

[t, x] = ode45(f, linspace(0, 12, 1201), x0);
y = (C*x.').';
uSamples = u(t);

stateIntegrand = sum((x*Qx).*x, 2);
inputIntegrand = Ru * (uSamples.^2);
outputIntegrand = sum((y*Wy).*y, 2);

stateEnergy = trapz(t, stateIntegrand);
inputEnergy = trapz(t, inputIntegrand);
J = stateEnergy + inputEnergy;
outputL2 = sqrt(trapz(t, outputIntegrand));
outputRMS = sqrt(trapz(t, outputIntegrand)/(t(end)-t(1)));
outputLinf = max(vecnorm(y, 2, 2));

fprintf('Weighted state energy   = %.8f\n', stateEnergy);
fprintf('Weighted input energy   = %.8f\n', inputEnergy);
fprintf('Performance J           = %.8f\n', J);
fprintf('Weighted output L2 norm = %.8f\n', outputL2);
fprintf('Weighted output RMS     = %.8f\n', outputRMS);
fprintf('Output L-infinity norm  = %.8f\n', outputLinf);

% Control System Toolbox object. This is useful for later lessons involving
% impulse/step response energies, Gramian-based measures, and optimal feedback.
sys = ss(A, B, C, D);
disp('State-space model:');
disp(sys);

% Optional Simulink creation: a minimal block diagram for xdot = A*x + B*u.
% Run this section only when Simulink is installed.
if license('test', 'Simulink')
    model = 'Chapter28_Lesson2_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);
    add_block('simulink/Sources/Sine Wave', [model '/SineInput']);
    add_block('simulink/Continuous/State-Space', [model '/StateSpacePlant']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);
    set_param([model '/StateSpacePlant'], 'A', mat2str(A), 'B', mat2str(B), ...
        'C', mat2str(C), 'D', mat2str(D), 'X0', mat2str(x0));
    set_param([model '/SineInput'], 'Amplitude', '0.5', 'Frequency', '3');
    add_line(model, 'SineInput/1', 'StateSpacePlant/1');
    add_line(model, 'StateSpacePlant/1', 'Scope/1');
    save_system(model, [model '.slx']);
    disp(['Created Simulink model: ' model '.slx']);
end
