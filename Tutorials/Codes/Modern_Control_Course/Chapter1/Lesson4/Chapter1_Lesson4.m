A = [ 0  1;
     -4 -2 ];
B = [ 1  0;
      0  1 ];
C = eye(2);          % outputs equal states
D = zeros(2, 2);

sys = ss(A, B, C, D);   % continuous-time state-space model

% Step responses from each input to each output
figure;
step(sys);              % MATLAB plots all 4 responses G_ij(s)

% Alternatively, step from input 1 only
figure;
step(sys(:, 1));        % all outputs, single input channel
      
