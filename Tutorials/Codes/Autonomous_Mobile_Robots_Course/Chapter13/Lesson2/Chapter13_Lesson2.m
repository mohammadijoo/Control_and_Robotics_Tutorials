% Chapter13_Lesson2.m
% Autonomous Mobile Robots — Chapter 13, Lesson 2
% Feature-Based vs Direct Methods (MATLAB)
%
% Requirements:
%   - Computer Vision Toolbox (for ORB/KLT/essential matrix helpers)

clear; clc;

% ---- User inputs ----
img1Path = "frame1.png";   % reference (t-1)
img2Path = "frame2.png";   % current (t)

I1 = im2gray(imread(img1Path));
I2 = im2gray(imread(img2Path));

% Camera intrinsics (example defaults; replace with your calibration)
fx = 525; fy = 525; cx = 319.5; cy = 239.5;
intrinsics = cameraIntrinsics([fx fy], [cx cy], size(I1));

fprintf("=== Feature-based (ORB + Essential + relativeCameraPose) ===\n");
points1 = detectORBFeatures(I1);
points2 = detectORBFeatures(I2);

[feat1, valid1] = extractFeatures(I1, points1);
[feat2, valid2] = extractFeatures(I2, points2);

idxPairs = matchFeatures(feat1, feat2, "MaxRatio", 0.75, "Unique", true);
matched1 = valid1(idxPairs(:,1));
matched2 = valid2(idxPairs(:,2));

% Estimate essential matrix with RANSAC
[E, inlierIdx] = estimateEssentialMatrix(matched1, matched2, intrinsics, ...
    "Confidence", 99.9, "MaxNumTrials", 2000);

in1 = matched1(inlierIdx);
in2 = matched2(inlierIdx);

% Recover relative pose (R, t up to scale)
[orient, loc] = relativeCameraPose(E, intrinsics, in1, in2);
t_unit = loc(:) / (norm(loc) + 1e-12);

disp("R = "); disp(orient);
disp("t (unit norm, scale unknown) = "); disp(t_unit');

fprintf("\n=== Direct (sparse KLT / photometric tracking) ===\n");
tracker = vision.PointTracker("MaxBidirectionalError", 2.0);
p0 = detectMinEigenFeatures(I1, "MinQuality", 0.01);
p0 = p0.Location;

initialize(tracker, p0, I1);
[p1, valid] = tracker(I2);

p0v = p0(valid,:);
p1v = p1(valid,:);

% Robust translation estimate via median flow
flow = p1v - p0v;
uv = median(flow, 1);
fprintf("Estimated pixel translation (median flow): u=%.3f, v=%.3f\n", uv(1), uv(2));

% Optional: estimate a similarity transform (rotation+translation) in the image plane
if size(p0v,1) >= 20
    tform = estimateGeometricTransform2D(p0v, p1v, "similarity", ...
        "MaxNumTrials", 2000, "Confidence", 99.0);
    disp("Estimated 2D similarity transform (image plane):");
    disp(tform.A);
end
