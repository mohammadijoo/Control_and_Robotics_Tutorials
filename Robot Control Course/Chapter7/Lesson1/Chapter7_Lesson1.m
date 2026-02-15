
% Nominal values
I_nom   = 0.5;
b_nom   = 0.05;
mgl_nom = 1.0;

% Uncertain parameters with +-20% relative deviation
I_u   = ureal('I',   I_nom,   'Percentage', 20);
b_u   = ureal('b',   b_nom,   'Percentage', 20);
mgl_u = ureal('mgl', mgl_nom, 'Percentage', 20);

% State-space representation: x = [q; qdot]
A = [0 1;
     0 -b_u/I_u];

B = [0;
     1/I_u];

E = [0;
     1/I_u];   % matched disturbance input

C = eye(2);
D = zeros(2, 2);

sys_unc = ss(A, [B E], C, D, 'StateName', {'q','qdot'}, ...
                      'InputName', {'tau','d'}, ...
                      'OutputName', {'q','qdot'});
