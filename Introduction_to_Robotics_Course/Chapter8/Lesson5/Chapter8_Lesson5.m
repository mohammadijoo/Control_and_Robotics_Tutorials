% (1) Acquire
img = imread('scene.png');
if size(img,3) == 3, img = rgb2gray(img); end

% (2) Preprocess
img_s = imgaussfilt(img, 1.2);

% (3) Feature extraction
corners = detectMinEigenFeatures(img_s, 'MinQuality', 0.01);
pts = corners.selectStrongest(200).Location;

% (4) Weighted least squares fusion
x1 = [1.2; 0.5];  x2 = [1.0; 0.9];
S1 = diag([0.04, 0.09]);  S2 = diag([0.16, 0.04]);

W1 = inv(S1); W2 = inv(S2);
xf = inv(W1 + W2) * (W1*x1 + W2*x2);

disp('Fused estimate:'); disp(xf);

% Simulink note:
% Use blocks: Image From File -> Gaussian Filter -> Corner Detector
% -> MATLAB Function block implementing xf formula.
