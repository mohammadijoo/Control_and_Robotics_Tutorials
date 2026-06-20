% Load point clouds
model = pcread("model_cloud.pcd");
scene = pcread("scene_cloud.pcd");

% Downsample
voxelSize = 0.005;
modelDS = pcdownsample(model, "gridAverage", voxelSize);
sceneDS = pcdownsample(scene, "gridAverage", voxelSize);

% ICP registration (point-to-point)
[tform, movingReg, rmse] = pcregistericp( ...
    modelDS, sceneDS, ...
    "Metric", "pointToPoint", ...
    "MaxIterations", 50, ...
    "Tolerance", [1e-6, 1e-6]);

R_icp = tform.T(1:3, 1:3);
t_icp = tform.T(1:3, 4);

disp("ICP rotation:");
disp(R_icp);
disp("ICP translation:");
disp(t_icp);

% SVD-based rigid transform from explicit correspondences
function [R, t] = rigidTransformSVD(P, Q)
% P, Q: N-by-3 matrices of corresponding points
assert(all(size(P) == size(Q)), "P and Q must have same size");
N = size(P, 1);

p_bar = mean(P, 1);
q_bar = mean(Q, 1);

P_centered = P - p_bar;
Q_centered = Q - q_bar;

H = P_centered' * Q_centered;
[U, S, V] = svd(H);

R = V * U';
if det(R) < 0
    V(:, 3) = -V(:, 3);
    R = V * U';
end

t = q_bar' - R * p_bar';
end
      
