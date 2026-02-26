% Chapter13_Lesson1.m
% Autonomous Mobile Robots (Control Engineering)
% Chapter 13 - Visual and Visual–Inertial SLAM (AMR Focus)
% Lesson 1  - Visual Odometry for Mobile Robots
%
% Feature-based monocular VO using MATLAB Computer Vision Toolbox:
%   - ORB detection / extraction
%   - Feature matching
%   - Essential matrix estimation (RANSAC)
%   - Relative pose recovery
%   - Trajectory chaining (up to unknown scale)
%
% AMR note:
%   Monocular VO has scale ambiguity. In practice, you metrify by:
%     (i) stereo/RGB-D, or (ii) wheel odometry / IMU / known camera height.
%
% Requirements:
%   - Computer Vision Toolbox
%   - (Optional) Simulink if you run the model-generation part at the end.
%
% Usage:
%   Set "imageFolder" below to a folder of sequential frames.
%   (Optionally) Set intrinsics.
%
% Output:
%   - vo_trajectory_xyz.csv saved inside imageFolder

clear; clc;

%% 1) Load image sequence
imageFolder = fullfile(pwd, "frames"); % <-- change this
imds = imageDatastore(imageFolder, "FileExtensions", {".png",".jpg",".jpeg",".bmp"});
if numel(imds.Files) < 2
    error("Need at least 2 images in imageFolder.");
end

I0 = readimage(imds, 1);
if size(I0,3) == 3, I0g = rgb2gray(I0); else, I0g = I0; end

%% 2) Camera intrinsics (edit for your camera)
fx = 525; fy = 525; cx = 319.5; cy = 239.5;
K = [fx 0 cx; 0 fy cy; 0 0 1];
intr = cameraIntrinsics([fx fy], [cx cy], size(I0g));

%% 3) Detect ORB features in first frame
p0 = detectORBFeatures(I0g, "ScaleFactor", 1.2, "NumLevels", 8);
[f0, v0] = extractFeatures(I0g, p0);
Tcw = eye(4); % world->camera
traj = cameraCenterWorld(Tcw);

%% 4) Process sequence
for k = 2:numel(imds.Files)
    I1 = readimage(imds, k);
    if size(I1,3) == 3, I1g = rgb2gray(I1); else, I1g = I1; end

    p1 = detectORBFeatures(I1g, "ScaleFactor", 1.2, "NumLevels", 8);
    [f1, v1] = extractFeatures(I1g, p1);

    idxPairs = matchFeatures(f0, f1, "Unique", true, "MaxRatio", 0.75, "MatchThreshold", 50);
    if size(idxPairs,1) < 20
        f0 = f1; v0 = v1;
        continue;
    end

    m0 = v0(idxPairs(:,1));
    m1 = v1(idxPairs(:,2));

    % Estimate Essential Matrix with RANSAC
    [E, inliers] = estimateEssentialMatrix(m0, m1, intr, ...
        "Confidence", 99.9, "MaxNumTrials", 2000);

    in0 = m0(inliers);
    in1 = m1(inliers);
    if numel(in0) < 20
        f0 = f1; v0 = v1;
        continue;
    end

    % Recover relative camera pose
    [orient, loc] = relativeCameraPose(E, intr, in0, in1);

    % relativeCameraPose returns orientation and location of camera2 in camera1 coordinates.
    % Convert to (world->cam) update: X2 = R*X1 + t
    R = orient';
    t = -R * loc'; % consistent with X2 = R X1 + t convention

    T12 = eye(4);
    T12(1:3,1:3) = R;
    T12(1:3,4) = t; % unit scale

    Tcw = T12 * Tcw;
    traj(end+1,:) = cameraCenterWorld(Tcw); %#ok<AGROW>

    if mod(k,10)==0 || k==numel(imds.Files)
        C = traj(end,:);
        fprintf("[INFO] frame %d/%d: Cw = %.3f, %.3f, %.3f\n", k-1, numel(imds.Files)-1, C(1), C(2), C(3));
    end

    f0 = f1; v0 = v1;
end

%% 5) Save trajectory
out = fullfile(imageFolder, "vo_trajectory_xyz.csv");
writematrix(traj, out);
fprintf("[DONE] trajectory saved to: %s\n", out);

%% 6) (Optional) Simulink: programmatically create a simple VO model skeleton
% This creates a Simulink model with a MATLAB Function block placeholder where
% you would implement a VO step (or call a System object).
%
% Uncomment if you have Simulink:
%
% modelName = "Chapter13_Lesson1_VO_Model";
% if bdIsLoaded(modelName), close_system(modelName,0); end
% new_system(modelName); open_system(modelName);
% add_block("simulink/User-Defined Functions/MATLAB Function", modelName + "/VO_Step");
% set_param(modelName + "/VO_Step", "Position", [200 120 380 200]);
% add_block("simulink/Sources/Constant", modelName + "/FrameIndex");
% set_param(modelName + "/FrameIndex", "Position", [50 140 120 180], "Value", "1");
% add_line(modelName, "FrameIndex/1", "VO_Step/1");
% save_system(modelName);
% fprintf("[INFO] Simulink model created: %s.slx\n", modelName);

%% Helper
function Cw = cameraCenterWorld(Tcw)
Rcw = Tcw(1:3,1:3);
tcw = Tcw(1:3,4);
Cw = -(Rcw')*tcw;
Cw = Cw(:).';
end
