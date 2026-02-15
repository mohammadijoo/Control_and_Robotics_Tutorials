% Physical parameters
J_eq = 0.01;
b_eq = 0.001;
K_t  = 0.1;
K_c  = 2.0;
N    = 50.0;

Kv    = (K_t * K_c) / (N * b_eq);
tau_m = J_eq / b_eq;

s  = tf('s');
Gp = Kv / (s * (tau_m * s + 1));

% P-only design for desired damping ratio
zeta = 0.7;
Kp   = 1 / (4 * zeta^2 * Kv * tau_m);

C = Kp;
T = feedback(C*Gp, 1);   % unity feedback closed loop

figure;
step(T);
grid on;
title('Position servo step response (MATLAB, P control)');

% --- Simulink model creation (simple unity-feedback loop) ---
model = 'servo_axis';
new_system(model);
open_system(model);

% Add blocks
add_block('simulink/Sources/Step',        [model '/ref']);
add_block('simulink/Math Operations/Sum', [model '/sum']);
add_block('simulink/Continuous/Transfer Fcn', [model '/Gp']);
add_block('simulink/Sinks/Scope',        [model '/scope']);
add_block('simulink/Commonly Used Blocks/Gain', [model '/Kp']);

% Set parameters
set_param([model '/Gp'], 'Numerator',   mat2str(Kv), ...
                         'Denominator', mat2str([tau_m 1 0])); % tau_m*s^2 + s
set_param([model '/Kp'], 'Gain', num2str(Kp));

% Connect lines: ref -> sum(+), feedback -, sum -> Kp -> Gp -> scope
add_line(model, 'ref/1',  'sum/1');
add_line(model, 'sum/1',  'Kp/1');
add_line(model, 'Kp/1',   'Gp/1');
add_line(model, 'Gp/1',   'scope/1');
add_line(model, 'Gp/1',   'sum/2', 'autorouting','on');  % negative feedback

set_param(model, 'StopTime', '0.5');
sim(model);
