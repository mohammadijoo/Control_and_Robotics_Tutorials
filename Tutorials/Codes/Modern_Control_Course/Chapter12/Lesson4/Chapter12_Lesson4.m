% Chapter12_Lesson4.m
% Modern Control — Chapter 12, Lesson 4
% Finite-horizon controllability Gramian, energy ellipsoid, and optional Simulink model.
% Requires base MATLAB. Control System Toolbox and Simulink sections are optional.

clear; clc; close all;

A = [-1 0; 0 -4];
B = [1; 0.08];
T = 2.0;
N = 4000;
dt = T/N;

% Wc(T)=int_0^T exp(A*s) B B' exp(A'*s) ds by trapezoidal quadrature.
W = zeros(2,2);
for k = 0:N
    s = k*dt;
    Phi = expm(A*s);
    G = Phi*B*B'*Phi';
    w = 1.0;
    if k == 0 || k == N
        w = 0.5;
    end
    W = W + w*G*dt;
end
W = 0.5*(W + W');

[V,D] = eig(W);
lambda = diag(D);
[lambda,idx] = sort(lambda,'ascend');
V = V(:,idx);

fprintf('Wc(T)=\n'); disp(W);
fprintf('Eigenvalues:\n'); disp(lambda.');
fprintf('Condition number = %.6g\n', lambda(end)/lambda(1));

E = @(delta) delta'*(W\delta);
e1 = [1;0];
e2 = [0;1];
fprintf('Energy to reach e1 = %.8f\n', E(e1));
fprintf('Energy to reach e2 = %.8f\n', E(e2));
fprintf('Best-axis energy = %.8f\n', 1/lambda(end));
fprintf('Worst-axis energy = %.8f\n', 1/lambda(1));

% Plot reachable energy ellipsoid delta' inv(W) delta <= rho^2.
rho = 1;
theta = linspace(0,2*pi,400);
circle = [cos(theta); sin(theta)];
ellipse = V*diag(rho*sqrt(lambda))*circle;
figure;
plot(ellipse(1,:), ellipse(2,:), 'LineWidth', 1.5); grid on; axis equal;
xlabel('state component x_1'); ylabel('state component x_2');
title('Finite-horizon controllability energy ellipsoid');
hold on;
for i = 1:2
    axisVector = V(:,i)*rho*sqrt(lambda(i));
    plot([0 axisVector(1)], [0 axisVector(2)], 'LineWidth', 2);
end

% Optional Control System Toolbox comparison for infinite-horizon stable systems.
if exist('ss','file') == 2 && exist('gram','file') == 2
    sys = ss(A,B,eye(2),zeros(2,1));
    Winf = gram(sys,'c');
    fprintf('Infinite-horizon Wc from Control System Toolbox:\n'); disp(Winf);
end

% Optional Simulink skeleton: create a state-space plant block for simulation.
% Feed a designed input signal u(t) to the input port and inspect state outputs.
if exist('new_system','file') == 2
    model = 'Chapter12_Lesson4_Simulink';
    if bdIsLoaded(model)
        close_system(model,0);
    end
    new_system(model);
    add_block('simulink/Sources/In1', [model '/u']);
    add_block('simulink/Continuous/State-Space', [model '/Plant']);
    add_block('simulink/Sinks/Out1', [model '/x']);
    set_param([model '/Plant'], 'A', mat2str(A), 'B', mat2str(B), ...
        'C', mat2str(eye(2)), 'D', mat2str(zeros(2,1)));
    add_line(model, 'u/1', 'Plant/1');
    add_line(model, 'Plant/1', 'x/1');
    save_system(model);
    fprintf('Created optional Simulink model: %s.slx\n', model);
end
