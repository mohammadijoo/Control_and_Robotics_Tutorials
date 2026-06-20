% Second-order plant and lead controller
zeta = 0.7; wn = 10;
s = tf('s');
P = 1 / (s^2 + 2*zeta*wn*s + wn^2);
K = 20; z = 2;
C = K * (s + z) / s;

L = C*P;
S = 1/(1+L);
T = L/(1+L);

% Weights
M_P = 1.5; A_P = 1e-3; wP = 1;
M_N = 2.0; A_N = 1e-3; wN = 50;
alpha = 1.5; wD = 30;

W_P = (s/M_P + wP) / (s + wP*A_P);
W_N = (s/M_N + wN) / (s + wN*A_N);
W_D = (s/alpha + wD) / (s + wD);

w = logspace(-2,3,500);
[magS,~,~]  = bode(S,  w);
[magT,~,~]  = bode(T,  w);
[magWP,~,~] = bode(W_P,w);
[magWN,~,~] = bode(W_N,w);
[magWD,~,~] = bode(W_D,w);

magS  = squeeze(magS);
magT  = squeeze(magT);
magWP = squeeze(magWP);
magWN = squeeze(magWN);
magWD = squeeze(magWD);

perfMetric   = magWP .* magS;
noiseMetric  = magWN .* magT;
robustMetric = magWD .* magT;
combined     = perfMetric + robustMetric;

fprintf('max |W_P S|      = %.3f\n', max(perfMetric));
fprintf('max |W_N T|      = %.3f\n', max(noiseMetric));
fprintf('max |W_D T|      = %.3f\n', max(robustMetric));
fprintf('max (|W_P S| + |W_D T|) = %.3f\n', max(combined));

% Simulink: represent P and C as blocks, inject disturbances and noise,
% and then log y,u to verify time-domain performance under worst-case signals.
