% ===== Code block 1 extracted from Chapter5/Lesson1.html =====
% Example: y''' + 3 y'' + 3 y' + 1 y = 2 u
a = [1; 3; 3];   % [a0; a1; a2]
b = 2;

n = length(a);
A = zeros(n);
A(1:n-1, 2:n) = eye(n-1);
A(n, :) = -a(:)';

B = zeros(n,1);
B(n) = b;

C = zeros(1,n);
C(1) = 1;

D = 0;

sys = ss(A,B,C,D);

t = linspace(0,10,5001);
u = ones(size(t));           % step input
x0 = [0;0;0];                % [y(0); y'(0); y''(0)]
[y,t_out,x] = lsim(sys,u,t,x0);

disp('A='), disp(A)
disp('Final y(tf)='), disp(y(end))

% ===== Code block 2 extracted from Chapter5/Lesson1.html =====
% Programmatic Simulink construction for the same third-order example
model = 'nth_order_chain';
new_system(model); open_system(model);

% Blocks
add_block('simulink/Sources/In1', [model '/u']);
add_block('simulink/Math Operations/Sum', [model '/Sum'], 'Inputs', '++++');
add_block('simulink/Math Operations/Gain', [model '/bGain'], 'Gain', '2');

add_block('simulink/Continuous/Integrator', [model '/Int3']);
add_block('simulink/Continuous/Integrator', [model '/Int2']);
add_block('simulink/Continuous/Integrator', [model '/Int1']);
add_block('simulink/Sinks/Out1', [model '/y']);

% Feedback gains for -a0*x1 - a1*x2 - a2*x3 (a0=1,a1=3,a2=3)
add_block('simulink/Math Operations/Gain', [model '/a0'], 'Gain', '-1');
add_block('simulink/Math Operations/Gain', [model '/a1'], 'Gain', '-3');
add_block('simulink/Math Operations/Gain', [model '/a2'], 'Gain', '-3');

% Layout positions (minimal; adjust as desired)
set_param([model '/u'], 'Position', [30 80 60 100]);
set_param([model '/bGain'], 'Position', [90 75 130 105]);
set_param([model '/Sum'], 'Position', [170 70 200 110]);
set_param([model '/Int3'], 'Position', [240 70 270 110]);
set_param([model '/Int2'], 'Position', [320 70 350 110]);
set_param([model '/Int1'], 'Position', [400 70 430 110]);
set_param([model '/y'], 'Position', [470 80 500 100]);

set_param([model '/a2'], 'Position', [240 150 270 180]);
set_param([model '/a1'], 'Position', [320 150 350 180]);
set_param([model '/a0'], 'Position', [400 150 430 180]);

% Connections: u -> bGain -> Sum -> Int3 -> Int2 -> Int1 -> y
add_line(model, 'u/1', 'bGain/1');
add_line(model, 'bGain/1', 'Sum/1');
add_line(model, 'Sum/1', 'Int3/1');
add_line(model, 'Int3/1', 'Int2/1');
add_line(model, 'Int2/1', 'Int1/1');
add_line(model, 'Int1/1', 'y/1');

% Feedback taps: x3 from Int3 output, x2 from Int2 output, x1 from Int1 output
add_line(model, 'Int3/1', 'a2/1');
add_line(model, 'Int2/1', 'a1/1');
add_line(model, 'Int1/1', 'a0/1');

% Gains feed into Sum remaining ports
add_line(model, 'a2/1', 'Sum/2');
add_line(model, 'a1/1', 'Sum/3');
add_line(model, 'a0/1', 'Sum/4');

save_system(model);
