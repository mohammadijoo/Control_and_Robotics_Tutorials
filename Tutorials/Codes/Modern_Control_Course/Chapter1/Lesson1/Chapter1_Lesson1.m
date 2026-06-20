% Physical parameters
m = 1.0;
c = 0.4;
k = 2.0;

% State-space matrices
A = [0 1;
     -k/m -c/m];
B = [0;
     1/m];
C = [1 0];  % position output
D = 0;

sys = ss(A, B, C, D);

% Step response (force input = unit step)
figure;
step(sys);
title('Mass-spring-damper: position response');

% In Simulink:
% 1. Drag a "State-Space" block into the model.
% 2. Set A, B, C, D matrices in the block parameters.
% 3. Connect input (force), output (position), and scopes as needed.
      
