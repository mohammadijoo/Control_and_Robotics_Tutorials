% Chapter13_Lesson4.m
% Visual/VIO SLAM robustness demo:
%  (1) monocular scale ambiguity via essential matrix (synthetic),
%  (2) affine brightness fit (gain/offset),
%  (3) blur score via Laplacian variance.
%
% Requires: Computer Vision Toolbox for estimateEssentialMatrix / relativeCameraPose.

function Chapter13_Lesson4()

% --- 1) Scale ambiguity demo (synthetic) ---
rng(0);
fx = 420; fy = 420; cx = 320; cy = 240;
K = [fx 0 cx; 0 fy cy; 0 0 1];

N = 200;
Xw = [ (rand(N,1)*4-2), (rand(N,1)*2-1), (rand(N,1)*4+4) ]; % z in [4,8]

R1 = eye(3); t1 = [0;0;0];

yaw = deg2rad(5);
R2 = [cos(yaw) 0 sin(yaw); 0 1 0; -sin(yaw) 0 cos(yaw)];
t2_metric = [0.20; 0.0; 0.80];

u1 = projectPoints(K,R1,t1,Xw);
u2 = projectPoints(K,R2,t2_metric,Xw);

noise = 0.5;
u1n = u1 + noise*randn(size(u1));
u2n = u2 + noise*randn(size(u2));

[E,inliers] = estimateEssentialMatrix(u1n,u2n,K,K,'Confidence',99.9,'MaxNumTrials',2000);
[orient, loc] = relativeCameraPose(E, K, u1n(inliers,:), u2n(inliers,:));

t_unit = loc(:); % camera2 location in cam1 coordinates (up to scale)
t_unit_norm = norm(t_unit);

fprintf('\n--- Monocular 2-view: translation scale ambiguity ---\n');
fprintf('Recovered translation direction (up to scale): [%g %g %g], ||t||=%g\n', t_unit, t_unit_norm);
fprintf('True metric translation: [%g %g %g], ||t||=%g\n', t2_metric, norm(t2_metric));

% Fake IMU metric delta-position:
delta_p_imu = t2_metric + 0.02*randn(3,1);
scale_hat = norm(delta_p_imu)/(t_unit_norm+1e-12);
t_metric_hat = scale_hat*t_unit;
fprintf('Estimated scale_hat=%g, metric t_hat=[%g %g %g]\n', scale_hat, t_metric_hat);

% --- 2) Photometric affine fit ---
img1 = zeros(240,320,'uint8');
img1 = insertText(img1,[60 110],'AMR','FontSize',72,'BoxOpacity',0,'TextColor','white');
img1 = rgb2gray(img1);

a_true = 1.2; b_true = -10;
img2 = uint8(min(max(a_true*double(img1)+b_true,0),255));
img2 = imgaussfilt(img2,2.5);

[a_hat, b_hat] = affineBrightnessFit(img1,img2);
fprintf('\n--- Lighting change fit ---\n');
fprintf('True (a,b)=(%g,%g), estimated=(%g,%g)\n', a_true,b_true,a_hat,b_hat);

% --- 3) Blur score ---
s1 = varLaplacian(img1);
s2 = varLaplacian(img2);
fprintf('\n--- Blur score ---\n');
fprintf('Blur score var(Laplacian): sharp=%g, blurred=%g\n', s1, s2);

end


function u = projectPoints(K,R,t,Xw)
Xc = (R*Xw' + t);
x = Xc(1:2,:)./Xc(3,:);
u = (K(1:2,1:2)*x + K(1:2,3)).';
end

function [a,b] = affineBrightnessFit(I1,I2)
x = double(I1(:));
y = double(I2(:));
A = [x, ones(size(x))];
theta = A\y;
a = theta(1); b = theta(2);
end

function s = varLaplacian(gray)
L = imfilter(double(gray), [0 1 0; 1 -4 1; 0 1 0], 'replicate');
s = var(L(:));
end
