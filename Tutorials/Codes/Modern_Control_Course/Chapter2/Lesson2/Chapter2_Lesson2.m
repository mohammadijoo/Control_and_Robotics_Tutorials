% ===== Code block 1 extracted from Chapter2/Lesson2.html =====
% T: R^3 -> R^2
A = [1 2 0; 0 -1 3];
x = [1; -2; 0.5];
Tx = A*x;

disp('A='); disp(A);
disp('x='); disp(x);
disp('T(x)=A*x='); disp(Tx);

% Composition with S: R^2 -> R^2
B = [0 1; -1 0];
BA = B*A;
disp('BA='); disp(BA);
disp('(S o T)(x)=BA*x='); disp(BA*x);
      

% ===== Code block 2 extracted from Chapter2/Lesson2.html =====
% Programmatic Simulink model: y = A x using Matrix Gain
model = 'LinearMap_Model';
new_system(model); open_system(model);

A = [1 2 0; 0 -1 3];

add_block('simulink/Sources/Constant', [model '/x'], 'Position', [50 80 120 120]);
set_param([model '/x'], 'Value', '[1; -2; 0.5]');

add_block('simulink/Math Operations/Matrix Gain', [model '/A'], 'Position', [180 75 260 125]);
set_param([model '/A'], 'Gain', 'A');

add_block('simulink/Sinks/Display', [model '/y'], 'Position', [320 80 380 120]);

add_line(model, 'x/1', 'A/1');
add_line(model, 'A/1', 'y/1');

save_system(model);
% Run simulation (Display block will show y)
sim(model);
      
