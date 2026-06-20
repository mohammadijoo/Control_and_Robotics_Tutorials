% System Dynamics — Chapter 14 (Nonlinear System Dynamics)
% Lesson 1: Sources and Types of Nonlinearities in Engineering Systems
%
% Programmatically builds a Simulink model that implements:
%   m xdd + c xd + k x + k3 x^3 + Fc*tanh(xd/vs) = b*sat(u)
%
% The model uses standard Simulink blocks: Integrator, Sum, Gain, Math Function,
% Saturation, Trigonometric Function, and a MATLAB Function block for tanh friction.
%
% Run this script in MATLAB. It creates and opens 'Chapter14_Lesson1_Simulink.slx'.

clear; clc;

model = 'Chapter14_Lesson1_Simulink';
if bdIsLoaded(model), close_system(model, 0); end
new_system(model); open_system(model);

% Parameters (in base workspace so blocks can use them)
m = 1.0; c = 0.4; k = 4.0; k3 = 8.0; Fc = 0.8; vs = 0.02; b = 1.0; umax = 1.5; %#ok<NASGU>
assignin('base','m',m); assignin('base','c',c); assignin('base','k',k);
assignin('base','k3',k3); assignin('base','Fc',Fc); assignin('base','vs',vs);
assignin('base','b',b); assignin('base','umax',umax);

% Layout helpers
x0 = 40; y0 = 50; dx = 120; dy = 70;

% Input u(t) = 1.2*sin(t) + 0.3*sin(3t)
add_block('simulink/Sources/Clock', [model '/t'], 'Position', [x0 y0 x0+30 y0+30]);
add_block('simulink/Math Operations/Gain', [model '/w1'], 'Gain', '1', 'Position', [x0+dx y0 x0+dx+40 y0+30]);
add_block('simulink/Math Operations/Gain', [model '/w3'], 'Gain', '3', 'Position', [x0+dx y0+dy x0+dx+40 y0+dy+30]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin1'], 'Operator', 'sin', 'Position', [x0+2*dx y0 x0+2*dx+50 y0+30]);
add_block('simulink/Math Operations/Trigonometric Function', [model '/sin3'], 'Operator', 'sin', 'Position', [x0+2*dx y0+dy x0+2*dx+50 y0+dy+30]);
add_block('simulink/Math Operations/Gain', [model '/a1'], 'Gain', '1.2', 'Position', [x0+3*dx y0 x0+3*dx+40 y0+30]);
add_block('simulink/Math Operations/Gain', [model '/a3'], 'Gain', '0.3', 'Position', [x0+3*dx y0+dy x0+3*dx+40 y0+dy+30]);
add_block('simulink/Math Operations/Sum', [model '/sum_u'], 'Inputs', '++', 'Position', [x0+4*dx y0+dy/2 x0+4*dx+40 y0+dy/2+40]);

% Saturation block
add_block('simulink/Discontinuities/Saturation', [model '/sat_u'], ...
    'UpperLimit', 'umax', 'LowerLimit', '-umax', ...
    'Position', [x0+5*dx y0+dy/2 x0+5*dx+60 y0+dy/2+40]);

% Dynamics: two integrators xdd -> xd -> x
add_block('simulink/Continuous/Integrator', [model '/int_v'], 'Position', [x0+7*dx y0 x0+7*dx+40 y0+40]);
add_block('simulink/Continuous/Integrator', [model '/int_x'], 'Position', [x0+8*dx y0 x0+8*dx+40 y0+40]);

% Compute spring: k*x + k3*x^3
add_block('simulink/Math Operations/Gain', [model '/k*x'], 'Gain', 'k', 'Position', [x0+8*dx y0+dy x0+8*dx+60 y0+dy+30]);
add_block('simulink/Math Operations/Math Function', [model '/x^3'], 'Operator', 'pow', 'Exponent', '3', ...
    'Position', [x0+8*dx y0+2*dy x0+8*dx+60 y0+2*dy+30]);
add_block('simulink/Math Operations/Gain', [model '/k3*x^3'], 'Gain', 'k3', 'Position', [x0+9*dx y0+2*dy x0+9*dx+70 y0+2*dy+30]);

add_block('simulink/Math Operations/Sum', [model '/sum_spring'], 'Inputs', '++', 'Position', [x0+10*dx y0+dy+25 x0+10*dx+40 y0+dy+65]);

% Friction: c*v + Fc*tanh(v/vs) via MATLAB Function block
add_block('simulink/Math Operations/Gain', [model '/c*v'], 'Gain', 'c', 'Position', [x0+8*dx y0+3*dy x0+8*dx+60 y0+3*dy+30]);
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/tanh_fric'], ...
    'Position', [x0+9*dx y0+3*dy x0+9*dx+130 y0+3*dy+60]);
set_param([model '/tanh_fric'], 'Script', sprintf([ ...
    'function y = f(v)\n' ...
    '%% Smoothed Coulomb friction: Fc*tanh(v/vs)\n' ...
    'Fc = evalin(''base'',''Fc'');\n' ...
    'vs = evalin(''base'',''vs'');\n' ...
    'y = Fc*tanh(v/vs);\n' ...
    'end\n' ...
]));

add_block('simulink/Math Operations/Sum', [model '/sum_fric'], 'Inputs', '++', 'Position', [x0+10*dx y0+3*dy x0+10*dx+40 y0+3*dy+40]);

% Force balance: (b*sat(u) - spring - fric)/m
add_block('simulink/Math Operations/Gain', [model '/b*u'], 'Gain', 'b', 'Position', [x0+6*dx y0+dy/2 x0+6*dx+50 y0+dy/2+40]);

add_block('simulink/Math Operations/Sum', [model '/sum_force'], 'Inputs', '+--', 'Position', [x0+11*dx y0+2*dy x0+11*dx+50 y0+2*dy+60]);
add_block('simulink/Math Operations/Gain', [model '/1/m'], 'Gain', '1/m', 'Position', [x0+12*dx y0+2*dy x0+12*dx+50 y0+2*dy+30]);

% Scopes
add_block('simulink/Sinks/Scope', [model '/Scope_x'], 'Position', [x0+9*dx y0-10 x0+9*dx+70 y0+30]);
add_block('simulink/Sinks/Scope', [model '/Scope_v'], 'Position', [x0+8*dx y0-10 x0+8*dx+70 y0+30]);

% Wiring
add_line(model,'t/1','w1/1');        add_line(model,'t/1','w3/1');
add_line(model,'w1/1','sin1/1');     add_line(model,'w3/1','sin3/1');
add_line(model,'sin1/1','a1/1');     add_line(model,'sin3/1','a3/1');
add_line(model,'a1/1','sum_u/1');    add_line(model,'a3/1','sum_u/2');
add_line(model,'sum_u/1','sat_u/1'); add_line(model,'sat_u/1','b*u/1');

add_line(model,'b*u/1','sum_force/1');
add_line(model,'1/m/1','int_v/1');
add_line(model,'int_v/1','int_x/1');

% Feedback signals
add_line(model,'int_x/1','k*x/1');
add_line(model,'int_x/1','x^3/1');
add_line(model,'x^3/1','k3*x^3/1');

add_line(model,'k*x/1','sum_spring/1');
add_line(model,'k3*x^3/1','sum_spring/2');

add_line(model,'int_v/1','c*v/1');
add_line(model,'int_v/1','tanh_fric/1');
add_line(model,'c*v/1','sum_fric/1');
add_line(model,'tanh_fric/1','sum_fric/2');

add_line(model,'sum_spring/1','sum_force/2');
add_line(model,'sum_fric/1','sum_force/3');
add_line(model,'sum_force/1','1/m/1');

% Scopes
add_line(model,'int_x/1','Scope_x/1');
add_line(model,'int_v/1','Scope_v/1');

set_param(model,'StopTime','20');
save_system(model);
open_system(model);
disp('Created and opened Chapter14_Lesson1_Simulink.slx');
