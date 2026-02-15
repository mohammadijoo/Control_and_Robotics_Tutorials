% MATLAB script: 2R planar arm with encoder offsets and linearized model
l1 = 1.0;
l2 = 1.0;
q_offset = [0.05; -0.02];

fk_ideal = @(q) [ ...
    l1*cos(q(1)) + l2*cos(q(1)+q(2)); ...
    l1*sin(q(1)) + l2*sin(q(1)+q(2)) ];

fk_offset = @(q) fk_ideal(q + q_offset);

fk_linear = @(q) [ ...
    l1 + l2; ...
    l1*q(1) + l2*(q(1)+q(2)) ];

q = deg2rad([10; 20]);
p_ideal  = fk_ideal(q);
p_offset = fk_offset(q);
p_lin    = fk_linear(q);

disp(p_ideal);
disp(p_offset);
disp(p_lin);

% Simulink note:
% A corresponding Simulink model can be built by:
% 1. Creating a "MATLAB Function" block implementing fk_ideal(q).
% 2. Adding blocks that inject constant offsets q_offset.
% 3. Using "Gain" blocks to form FK_LINEAR(q) and comparing outputs.
% This visually demonstrates the impact of different modeling assumptions.
      
