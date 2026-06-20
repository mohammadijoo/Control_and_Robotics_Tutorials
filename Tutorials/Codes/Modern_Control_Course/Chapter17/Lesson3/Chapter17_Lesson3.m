% Chapter17_Lesson3.m
% Diagonal modal form for a continuous-time LTI system with distinct eigenvalues.
%
% MATLAB libraries/toolboxes:
%   eig, inv, expm are available in base MATLAB.
%   ss, initial, modalreal require Control System Toolbox.
%   Simulink implementation can use a State-Space block with A,B,C,D or
%   separate Integrator/Gain/Sum blocks for each modal state.

clear; clc;

A = [ 0.0  1.0  0.0;
     -2.0 -3.0  0.0;
      0.5  0.0 -4.0];
B = [0.0; 1.0; 0.0];
C = [1.0 0.0 1.0];
D = 0.0;
x0 = [1.0; 0.0; -0.5];

% Modal transformation x = V z, z = inv(V) x
[V,Lambda] = eig(A);
Vinv = inv(V);
Bm = Vinv * B;
Cm = C * V;
z0 = Vinv * x0;

disp('Eigenvalues:');
disp(diag(Lambda));
disp('Modal A matrix Lambda = inv(V)*A*V:');
disp(Vinv * A * V);
disp('Modal input matrix Bm = inv(V)*B:');
disp(Bm);
disp('Modal output matrix Cm = C*V:');
disp(Cm);
disp('Condition number of V:');
disp(cond(V));

% Zero-input response from physical and modal coordinates
t = linspace(0,5,101);
yPhysical = zeros(size(t));
yModal = zeros(size(t));
for k = 1:length(t)
    xPhysical = expm(A*t(k)) * x0;
    zModal = expm(Lambda*t(k)) * z0;
    xFromModal = V * zModal;
    yPhysical(k) = C * xPhysical;
    yModal(k) = C * xFromModal;
end

fprintf('Agreement norm between physical and modal outputs: %.3e\n', norm(yPhysical-yModal));

% Control System Toolbox workflow
sys = ss(A,B,C,D);
try
    [sysModal, blocks] = modalreal(sys);
    disp('modalreal block information:');
    disp(blocks);
    disp('Modal realization returned by modalreal:');
    sysModal
catch ME
    disp('modalreal is unavailable or Control System Toolbox is missing.');
    disp(ME.message);
end

% Simulink note:
% To build the modal model manually, create one Integrator block for each z_i:
%   dz_i/dt = lambda_i z_i + row_i(Bm) u
% Then compute y = Cm z + D u using Gain and Sum blocks.
% For complex conjugate eigenvalues, use real 2-by-2 modal blocks instead.
