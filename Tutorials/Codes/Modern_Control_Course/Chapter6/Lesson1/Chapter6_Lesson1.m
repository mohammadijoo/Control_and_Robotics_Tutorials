% Example matrices
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

% Build state-space model
sys = ss(A,B,C,D);

% Convert to transfer function
G = tf(sys)

% Evaluate G(s) at s = j*2
s = 1j*2;
Gj2 = evalfr(sys, s)

% For SISO, you can also extract numerator/denominator explicitly
[num, den] = ss2tf(A,B,C,D);
num, den
      
