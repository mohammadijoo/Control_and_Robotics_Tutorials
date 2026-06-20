% ===== Code block 1 extracted from Chapter6/Lesson3.html =====
% Original realization
A = [0 1; -2 -3];
B = [0; 1];
C = [2 1];
D = 0;

sys1 = ss(A,B,C,D);
tf1 = tf(sys1)

% Similarity transform z = T x
T  = [1 1; 0 1];
Ti = inv(T);

A2 = T*A*Ti;
B2 = T*B;
C2 = C*Ti;
D2 = D;

sys2 = ss(A2,B2,C2,D2);
tf2 = tf(sys2)

% Numeric check on frequency response
w = logspace(-2,2,50);
[mag1,ph1] = bode(sys1,w);
[mag2,ph2] = bode(sys2,w);
maxMagErr = max(abs(mag1(:)-mag2(:)));
maxPhErr  = max(abs(ph1(:)-ph2(:)));
disp([maxMagErr, maxPhErr])

% Append hidden dynamics
Ah = -5;                 % hidden pole
Aaug = blkdiag(A, Ah);
Baug = [B; 0];
Caug = [C 0];
Daug = D;

sysAug = ss(Aaug, Baug, Caug, Daug);
tfAug = tf(sysAug)
      

% ===== Code block 2 extracted from Chapter6/Lesson3.html =====
% Build a Simulink model to compare sys1 and sys2 step responses
model = 'ss_similarity_demo';
new_system(model); open_system(model);

add_block('simulink/Sources/Step', [model '/Step']);
add_block('simulink/Continuous/State-Space', [model '/SS1']);
add_block('simulink/Continuous/State-Space', [model '/SS2']);
add_block('simulink/Sinks/Scope', [model '/Scope']);

% Set matrices (as strings)
set_param([model '/SS1'], 'A', mat2str(A),  'B', mat2str(B),  'C', mat2str(C),  'D', mat2str(D));
set_param([model '/SS2'], 'A', mat2str(A2), 'B', mat2str(B2), 'C', mat2str(C2), 'D', mat2str(D2));

% Wire the model
add_line(model, 'Step/1', 'SS1/1');
add_line(model, 'Step/1', 'SS2/1');
add_line(model, 'SS1/1', 'Scope/1');
add_line(model, 'SS2/1', 'Scope/2');

set_param(model, 'StopTime', '5');
sim(model);
      
