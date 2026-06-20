% ===== Code block 1 extracted from Chapter5/Lesson4.html =====
% Example 1: m*ydd + c*yd + k*y = u
m = 2.0; c = 3.0; k = 5.0;

A = [0 1;
    -k/m  -c/m];
B = [0;
     1/m];

% Outputs: y1 = y, y2 = ydd = -(k/m)*x1 -(c/m)*x2 + (1/m)*u
C = [ 1    0;
     -k/m -c/m ];
D = [0;
     1/m];

sys = ss(A,B,C,D);
disp(sys)
      

% ===== Code block 2 extracted from Chapter5/Lesson4.html =====
% Example 2: y''' + a2 y'' + a1 y' + a0 y = b1 u1 + b2 u2
a0 = 2.0; a1 = 3.0; a2 = 1.0;
b1 = 5.0; b2 = -1.0;

A = [0 1 0;
     0 0 1;
    -a0 -a1 -a2];
B = [0 0;
     0 0;
     b1 b2];

% Outputs: y1 = y = x1, y2 = y' = x2
C = [1 0 0;
     0 1 0];
D = zeros(2,2);

sys = ss(A,B,C,D);
disp(sys)
      
