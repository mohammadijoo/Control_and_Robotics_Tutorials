
% DC motor approximate first-order model from Lesson:
R = 2; J = 1e-3; b = 1e-4; ke = 0.05; kt = 0.05;
num = kt;
den = [R*J, (R*b + ke*kt)];
Gm = tf(num, den);   % omega / voltage

% Discretize for digital control
Ts = 1e-3;
Gmz = c2d(Gm, Ts, 'zoh');

% Simple discrete PD on position (theta = integral of omega)
% Use state-space augmentation: x1=theta, x2=omega
A = [0 1; 0 -den(2)/den(1)];
B = [0; num/den(1)];
C = [1 0]; D = 0;
sys = ss(A,B,C,D);
sysz = c2d(sys, Ts);

Kp = 2.0; Kd = 0.02;

% Simulate closed-loop with quantized encoder
N = 4096; delta = 2*pi/N;
x = [0;0]; ref = 1.0;
for k=1:500
    theta = x(1); omega = x(2);
    theta_q = delta*round(theta/delta);
    e = ref - theta_q;
    u = Kp*e - Kd*omega;
    u = max(-12, min(12, u));
    x = sysz.A*x + sysz.B*u;
end
disp(x(1));

% Simulink idea:
% blocks: Zero-Order Hold -> Discrete PD -> Saturation -> Motor Transfer Fcn -> Integrator -> Encoder Quantizer
      