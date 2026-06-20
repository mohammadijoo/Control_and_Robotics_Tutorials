% Chapter17_Lesson4.m
% Transformations Between Physical and Modal Coordinates
%
% Continuous-time LTI system:
%   xdot = A*x + B*u,     y = C*x + D*u
% Coordinate transformation:
%   x = T*z,              z = inv(T)*x
% Modal model:
%   zdot = Lambda*z + Bm*u,    y = Cm*z + D*u

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

% MATLAB eig returns A*V = V*Lambda.
[T, Lambda] = eig(A);
Tinv = inv(T);
Bm = Tinv * B;
Cm = C * T;

fprintf('T (eigenvector matrix):\n'); disp(T);
fprintf('Lambda = inv(T)*A*T:\n'); disp(Tinv * A * T);
fprintf('Bm = inv(T)*B:\n'); disp(Bm);
fprintf('Cm = C*T:\n'); disp(Cm);

x0 = [2; -1];
z0 = Tinv * x0;
x0_recovered = T * z0;
fprintf('z0 = inv(T)*x0:\n'); disp(z0);
fprintf('T*z0:\n'); disp(x0_recovered);

% Verify transfer-function invariance.
s = 1 + 0.5i;
G_phys = C * inv(s*eye(size(A)) - A) * B + D;
G_modal = Cm * inv(s*eye(size(Lambda)) - Lambda) * Bm + D;
fprintf('G_phys(s)  = %.8f%+.8fi\n', real(G_phys), imag(G_phys));
fprintf('G_modal(s) = %.8f%+.8fi\n', real(G_modal), imag(G_modal));
fprintf('Difference norm = %.3e\n', norm(G_phys - G_modal));

% Simulink note: if Simulink is installed, this block creates a small model
% containing physical and modal State-Space blocks driven by the same input.
createSimulinkModel = false;
if createSimulinkModel
    model = 'Chapter17_Lesson4_Simulink';
    new_system(model); open_system(model);
    add_block('simulink/Sources/Step', [model '/Step']);
    add_block('simulink/Continuous/State-Space', [model '/Physical_State_Space']);
    add_block('simulink/Continuous/State-Space', [model '/Modal_State_Space']);
    set_param([model '/Physical_State_Space'], 'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D');
    set_param([model '/Modal_State_Space'], 'A', 'Lambda', 'B', 'Bm', 'C', 'Cm', 'D', 'D');
    add_block('simulink/Sinks/Scope', [model '/Scope_Physical']);
    add_block('simulink/Sinks/Scope', [model '/Scope_Modal']);
    add_line(model, 'Step/1', 'Physical_State_Space/1');
    add_line(model, 'Step/1', 'Modal_State_Space/1');
    add_line(model, 'Physical_State_Space/1', 'Scope_Physical/1');
    add_line(model, 'Modal_State_Space/1', 'Scope_Modal/1');
    save_system(model);
end
