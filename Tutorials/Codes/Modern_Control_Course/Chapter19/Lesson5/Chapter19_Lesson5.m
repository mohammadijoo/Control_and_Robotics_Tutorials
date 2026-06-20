% Chapter19_Lesson5.m
% Physical Interpretation of Decomposed Subsystems in Kalman Decomposition
% MATLAB / Simulink-oriented script. Uses Control System Toolbox if available,
% but includes local controllability and observability matrix builders.

clear; clc;

% State order: [x_co1, x_co2, x_c_no, x_no_o, x_no_no]
A = [ 0.0  1.0  0.0  0.2  0.0;
     -2.0 -3.0  0.0  0.0  0.0;
      0.0  0.3 -4.0  0.1  0.0;
      0.0  0.0  0.0 -0.5  0.0;
      0.0  0.0  0.0  0.0  0.2 ];
B = [0; 1; 1; 0; 0];
C = [1 0 0 1 0];
D = 0;

Wc = local_ctrb(A, B);
Wo = local_obsv(A, C);

fprintf('rank(Wc) = %d out of %d\n', rank(Wc), size(A,1));
fprintf('rank(Wo) = %d out of %d\n', rank(Wo), size(A,1));
fprintf('Reachable dimension should be 3: co plus c_no.\n');
fprintf('Observable dimension should be 3: co plus no_o.\n');

% If Control System Toolbox is installed, build the state-space model.
if exist('ss','file') == 2
    sys = ss(A, B, C, D);
    disp('State-space model:');
    disp(sys);
    disp('Zero-initial transfer function:');
    disp(tf(sys));
end

% Zero-initial step response: x_c_no is internally excited, but hidden from y.
dt = 0.002; tFinal = 6; t = 0:dt:tFinal;
x = zeros(5, numel(t)); y = zeros(1, numel(t));
for k = 1:numel(t)-1
    u = 1;
    y(k) = C*x(:,k) + D*u;
    x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u);
end
y(end) = C*x(:,end) + D;
fprintf('Final step-response output y = %.6f\n', y(end));
fprintf('Final hidden x_c_no = %.6f\n', x(3,end));

% Simulink note:
% Use a State-Space block with A, B, C, D defined above. Feed a Step block into
% the input and route y to a Scope. To inspect hidden states, enable state
% logging or replace the block with an integrator-level realization.

function Wc = local_ctrb(A, B)
    n = size(A,1);
    Wc = [];
    Ak = eye(n);
    for k = 1:n
        Wc = [Wc Ak*B]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

function Wo = local_obsv(A, C)
    n = size(A,1);
    Wo = [];
    Ak = eye(n);
    for k = 1:n
        Wo = [Wo; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end
