% ===== Code block 1 extracted from Chapter6/Lesson5.html =====
% Example: (s+1)/((s+1)(s+2)) = 1/(s+2)
s = tf('s');
G_nonreduced = (s+1)/((s+1)*(s+2));
G_reduced    = 1/(s+2);

% State-space realizations
sys_nonred = ss(G_nonreduced);   % may contain cancellation depending on realization/tolerance
sys_red    = ss(G_reduced);

% Minimal reduction (numerical pole-zero cancellation)
sys_nonred_min = minreal(sys_nonred);

disp('Orders:')
disp(size(sys_nonred.A,1))
disp(size(sys_red.A,1))
disp(size(sys_nonred_min.A,1))

% Verify transfer functions
tf(sys_red)
tf(sys_nonred_min)

% Compare frequency responses
w = logspace(-2,2,50);
[mag1,ph1] = bode(sys_red,w);
[mag2,ph2] = bode(sys_nonred_min,w);
maxMagErr = max(abs(mag1(:)-mag2(:)));
maxPhErr  = max(abs(ph1(:)-ph2(:)));
disp(maxMagErr)
disp(maxPhErr)
      

% ===== Code block 2 extracted from Chapter6/Lesson5.html =====
% Minimal state-space for 1/(s+2)
A = -2; B = 1; C = 1; D = 0;

% Augment with hidden state xh_dot = -1*xh, no coupling
A_aug = [A 0; 0 -1];
B_aug = [B; 0];
C_aug = [C 0];
D_aug = D;

% Create a Simulink model
mdl = 'min_vs_nonmin_demo';
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Step', [mdl '/u']);
add_block('simulink/Continuous/State-Space', [mdl '/SS_min']);
add_block('simulink/Continuous/State-Space', [mdl '/SS_nonmin']);
add_block('simulink/Sinks/Scope', [mdl '/Scope']);

set_param([mdl '/SS_min'],   'A', mat2str(A),     'B', mat2str(B),     'C', mat2str(C),     'D', mat2str(D));
set_param([mdl '/SS_nonmin'],'A', mat2str(A_aug), 'B', mat2str(B_aug), 'C', mat2str(C_aug), 'D', mat2str(D_aug));

add_line(mdl,'u/1','SS_min/1');
add_line(mdl,'u/1','SS_nonmin/1');
add_line(mdl,'SS_min/1','Scope/1');
add_line(mdl,'SS_nonmin/1','Scope/2');

set_param(mdl, 'StopTime', '10');
sim(mdl);
      
