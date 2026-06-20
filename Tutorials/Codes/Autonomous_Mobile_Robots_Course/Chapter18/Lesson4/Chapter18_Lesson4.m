% Chapter18_Lesson4.m
% Weather/Lighting Effects on Perception (Outdoor and Field AMR)

rng(5);

%% Part A: Synthetic camera degradation
n = 400;
x = linspace(0,1,n);

J = 0.25 + 0.35*(x >= 0.2) + 0.15*sin(14*pi*x) + 0.20*(x >= 0.58);
J = min(max(J,0),1);

d = 5 + 30*x;
beta = 0.07;
A = 0.85;

t = exp(-beta .* d);
I_fog = J .* t + A .* (1 - t);

exposure = 0.28;
readSigma = 0.015;
shotSigma = sqrt(max(I_fog*exposure,1e-6)) * 0.05;
I_low = exposure .* I_fog + randn(1,n) .* (shotSigma + readSigma);
I_low = min(max(I_low,0),1);

gamma = 0.6;
I_enh = I_low .^ gamma;
I_enh = (I_enh - min(I_enh)) / (max(I_enh) - min(I_enh) + 1e-12);

michelson = @(s) (max(s)-min(s)) / (max(s)+min(s)+1e-12);
fprintf('Contrast clear=%.4f, degraded=%.4f, enhanced=%.4f\n', michelson(J), michelson(I_low), michelson(I_enh));

%% Part B: Confidence-aware EKF
dt = 0.1; T = 250;
F = [1 dt; 0 1];
Q = [1e-4 0; 0 3e-3];
H = [1 0];
Rcam0 = 0.60^2;
Rlidar0 = 0.25^2;

xTrue = zeros(2,T); xTrue(:,1) = [0; 0.35];
for k = 2:T
    xTrue(:,k) = F*xTrue(:,k-1) + mvnrnd([0;0], Q)';
end

xA = [0;0]; PA = diag([1 0.5]);
xF = [0;0]; PF = diag([1 0.5]);
estA = zeros(2,T); estF = zeros(2,T);

cameraConf = @(fog,lux,rain) min(max((lux^0.7)*exp(-2.2*fog)*exp(-1.4*rain), 0.05), 1.0);
lidarConf  = @(fog,rain)     min(max(exp(-1.3*fog)*exp(-0.6*rain), 0.10), 1.0);

for k = 1:T
    fog = 0.10 + 0.35*exp(-0.5*((k-120)/22)^2);
    rain = 0.05 + 0.25*exp(-0.5*((k-180)/14)^2);
    lux = 1.0; if k >= 140, lux = 0.25; end

    qc = cameraConf(fog,lux,rain);
    ql = lidarConf(fog,rain);

    zCam = xTrue(1,k) + sqrt(Rcam0/qc)*randn;
    zLidar = xTrue(1,k) + sqrt(Rlidar0/ql)*randn;

    xA = F*xA; PA = F*PA*F' + Q;
    xF = F*xF; PF = F*PF*F' + Q;

    [xA, PA] = ekf1DUpdate(xA, PA, zCam, H, Rcam0/qc);
    [xA, PA] = ekf1DUpdate(xA, PA, zLidar, H, Rlidar0/ql);

    [xF, PF] = ekf1DUpdate(xF, PF, zCam, H, Rcam0);
    [xF, PF] = ekf1DUpdate(xF, PF, zLidar, H, Rlidar0);

    estA(:,k) = xA; estF(:,k) = xF;
end

rmseA = sqrt(mean((estA(1,:) - xTrue(1,:)).^2));
rmseF = sqrt(mean((estF(1,:) - xTrue(1,:)).^2));
fprintf('Adaptive RMSE=%.4f, Fixed-R RMSE=%.4f\n', rmseA, rmseF);

figure; plot(1:T, xTrue(1,:), 1:T, estA(1,:), 1:T, estF(1,:));
legend('Truth','Adaptive EKF','Fixed-R EKF'); grid on;
xlabel('Step'); ylabel('Position'); title('Weather-aware covariance adaptation');

function [x, P] = ekf1DUpdate(x, P, z, H, R)
    y = z - H*x;
    S = H*P*H' + R;
    K = (P*H') / S;
    x = x + K*y;
    I = eye(size(P));
    P = (I - K*H)*P*(I - K*H)' + K*R*K';
end
