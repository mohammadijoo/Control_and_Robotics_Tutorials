% Nominal first-order plant for a robot joint actuator
K0   = 10;      % rad/(V*s)
tau0 = 0.05;    % s
s = tf('s');
G0 = K0 / (tau0*s + 1);

alpha = 0.2;    % 20% gain
beta  = 0.1;    % 10% time constant

w = logspace(0,3,300);   % 1 to 1000 rad/s

% Frequency response of nominal plant
[mag0,ph0] = bode(G0,w);
mag0 = squeeze(mag0);

% Sample several uncertain plants
nSamples = 50;
DeltaM_bound = zeros(size(w));

for k = 1:nSamples
    K   = K0  * (1 + alpha*(2*rand-1));
    tau = tau0* (1 + beta *(2*rand-1));
    G   = K / (tau*s + 1);

    mag = squeeze(bode(G,w));
    DeltaM = abs(mag - mag0) ./ mag0;
    DeltaM_bound = max(DeltaM_bound, DeltaM);
end

% Simple hand-designed multiplicative weight (first order plus offset)
kw = 0.3;
wc = 1/tau0;
eps = 0.05;
WM = kw*s/(s+wc) + eps;

% Compare |W_M(jw)| to empirical bound
[magW,~] = bode(WM,w);
magW = squeeze(magW);

figure;
loglog(w, DeltaM_bound, 'LineWidth', 1.5); hold on;
loglog(w, magW, '--', 'LineWidth', 1.5);
grid on;
xlabel('\omega [rad/s]');
ylabel('Magnitude');
legend('|Delta_M(j\omega)| bound', '|W_M(j\omega)|');

title('Multiplicative uncertainty bound vs weight');

% In Simulink, G0 and W_M can be implemented using Transfer Fcn blocks, and
% Delta as a gain in [-1,1] to visualize worst-case plant trajectories.
