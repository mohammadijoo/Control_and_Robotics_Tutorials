% ===== Code block 1 extracted from Chapter1/Lesson2.html =====
% Classical LTI plant
zeta = 0.3;
wn   = 2.0;

num = wn^2;
den = [1, 2*zeta*wn, wn^2];
G = tf(num, den);

% Closed-loop with unity feedback
T = feedback(G, 1);

t = linspace(0, 10, 1000);
[y_step, t_out] = step(T, t);

figure;
plot(t_out, y_step);
grid on;
xlabel('t');
ylabel('y(t)');
title('Unity-feedback step response (LTI transfer function)');
      

% ===== Code block 2 extracted from Chapter1/Lesson2.html =====
% Sketch of setting up a Simulink model programmatically (conceptual)
model = 'tv_saturating_plant';
new_system(model);
open_system(model);

% Add integrators, sum blocks, saturation, etc. (block positions omitted)
add_block('simulink/Commonly Used Blocks/Integrator', [model '/Int1']);
add_block('simulink/Commonly Used Blocks/Integrator', [model '/Int2']);
add_block('simulink/Math Operations/Sum', [model '/Sum']);
add_block('simulink/Discontinuities/Saturation', [model '/Sat']);

% ... connect lines and set parameters, then simulate with sim(model) ...
      
