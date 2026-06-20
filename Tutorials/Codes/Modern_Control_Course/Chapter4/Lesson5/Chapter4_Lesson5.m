% ===== Code block 1 extracted from Chapter4/Lesson5.html =====
% Parameters
m = 1.0; b = 0.4; k = 4.0;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

sys_ss = ss(A,B,C,D);

% Transfer function from state-space
sys_tf = tf(sys_ss);
disp('G(s) from ss:'); disp(sys_tf);

% Polynomial form transfer function
sys_tf_poly = tf(1, [m b k]);

% Compare frequency response numerically
w = logspace(-2,2,10);
[mag1, ph1] = bode(sys_ss, w);  % state-space route
[mag2, ph2] = bode(sys_tf_poly, w); % TF route

maxMagDiff = max(abs(squeeze(mag1) - squeeze(mag2)));
maxPhDiff  = max(abs(squeeze(ph1)  - squeeze(ph2)));
fprintf('Max mag diff: %g, Max phase diff (deg): %g\n', maxMagDiff, maxPhDiff);

% Step response comparison (forced, IC=0)
t = linspace(0,20,2000);
[y_ss, t_ss] = step(sys_ss, t);
[y_tf, t_tf] = step(sys_tf_poly, t);
fprintf('Max |y_ss - y_tf|: %g\n', max(abs(y_ss - y_tf)));

% Demonstrate role of initial conditions (state-space only)
x0 = [1; 0];  % initial displacement
u  = ones(size(t)); % step input
[y_forced, t_forced, x_forced] = lsim(sys_ss, u, t, x0);
disp('With x0 != 0, output includes natural response term not contained in G(s) alone.');

% ===== Code block 2 extracted from Chapter4/Lesson5.html =====
% Build two Simulink models: one SS, one TF, compare outputs
m = 1.0; b = 0.4; k = 4.0;
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

% Model 1: State-Space block
mdl1 = 'ss_model_cmp';
new_system(mdl1); open_system(mdl1);
add_block('simulink/Sources/Step', [mdl1 '/Step'], 'Time','0','Before','0','After','1');
add_block('simulink/Continuous/State-Space', [mdl1 '/SS']);
set_param([mdl1 '/SS'], 'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));
add_block('simulink/Sinks/To Workspace', [mdl1 '/Yss'], 'VariableName','y_ss','SaveFormat','Array');
add_line(mdl1, 'Step/1', 'SS/1');
add_line(mdl1, 'SS/1', 'Yss/1');
set_param(mdl1, 'StopTime','20');

% Model 2: Transfer Fcn block
mdl2 = 'tf_model_cmp';
new_system(mdl2); open_system(mdl2);
add_block('simulink/Sources/Step', [mdl2 '/Step'], 'Time','0','Before','0','After','1');
add_block('simulink/Continuous/Transfer Fcn', [mdl2 '/TF']);
set_param([mdl2 '/TF'], 'Numerator', mat2str(1), 'Denominator', mat2str([m b k]));
add_block('simulink/Sinks/To Workspace', [mdl2 '/Ytf'], 'VariableName','y_tf','SaveFormat','Array');
add_line(mdl2, 'Step/1', 'TF/1');
add_line(mdl2, 'TF/1', 'Ytf/1');
set_param(mdl2, 'StopTime','20');

% Simulate both
sim(mdl1);
sim(mdl2);

% Compare results in MATLAB
t_ss = y_ss(:,1); out_ss = y_ss(:,2);
t_tf = y_tf(:,1); out_tf = y_tf(:,2);
fprintf('Max |y_ss - y_tf| (aligned by interpolation): %g\n', max(abs(out_ss - interp1(t_tf, out_tf, t_ss))));
