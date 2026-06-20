% ===== Code block 1 extracted from Chapter2/Lesson4.html =====
% Similarity transformation in MATLAB
A = [2 1; 0 3];
T = [1 1; 0 1];

A_tilde = T \ (A*T);   % solves T*A_tilde = A*T  => A_tilde = T^{-1} A T

eigA = eig(A);
eigAt = eig(A_tilde);

disp("A_tilde ="); disp(A_tilde);
disp("eig(A) ="); disp(eigA);
disp("eig(A_tilde) ="); disp(eigAt);

disp("trace equal?"); disp(abs(trace(A)-trace(A_tilde)) < 1e-10);
disp("det equal?");   disp(abs(det(A)-det(A_tilde)) < 1e-10);
      

% ===== Code block 2 extracted from Chapter2/Lesson4.html =====
% Programmatically create a Simulink model for coordinate conversion
T = [1 1; 0 1];
Tinv = inv(T);

mdl = "coord_change_demo";
new_system(mdl); open_system(mdl);

add_block("simulink/Sources/Sine Wave", mdl + "/z1");
add_block("simulink/Sources/Sine Wave", mdl + "/z2");
set_param(mdl + "/z2", "Phase", "1.2");

add_block("simulink/Signal Routing/Mux", mdl + "/Mux");
add_block("simulink/Math Operations/Gain", mdl + "/Gain_T");
set_param(mdl + "/Gain_T", "Gain", "T", "Multiplication", "Matrix(K*u)");

add_block("simulink/Math Operations/Gain", mdl + "/Gain_Tinv");
set_param(mdl + "/Gain_Tinv", "Gain", "Tinv", "Multiplication", "Matrix(K*u)");

add_block("simulink/Sinks/Scope", mdl + "/Scope_x");
add_block("simulink/Sinks/Scope", mdl + "/Scope_zrec");

add_line(mdl, "z1/1", "Mux/1");
add_line(mdl, "z2/1", "Mux/2");
add_line(mdl, "Mux/1", "Gain_T/1");
add_line(mdl, "Gain_T/1", "Scope_x/1");
add_line(mdl, "Gain_T/1", "Gain_Tinv/1");
add_line(mdl, "Gain_Tinv/1", "Scope_zrec/1");

set_param(mdl, "StopTime", "10");
save_system(mdl);
      
