% Load RGB image
rgb = imread("scene.png");

% Load pre-trained semantic segmentation network (e.g., DeepLab v3+)
load("trainedSegNet.mat","net");

% Perform pixel-wise classification
[C, scores] = semanticseg(rgb, net);

% Visualize one class mask, e.g., "targetObject"
targetClass = "targetObject";
mask = C == targetClass;
imshowpair(rgb, mask, "montage");

% Suppose we have a registered depth image and camera intrinsics K
depth = imread("depth.png");      % depth in meters stored as single
depth = single(depth);
K = [fx 0 cx; 0 fy cy; 0 0 1];

% Back-project mask pixels into 3D camera frame
[H, W] = size(mask);
[u, v] = meshgrid(0:W-1, 0:H-1);
u = single(u); v = single(v);

idx = find(mask);
u_vec = u(idx);
v_vec = v(idx);
d_vec = depth(idx);

pix = [u_vec.'; v_vec.'; ones(1, numel(u_vec), "single")];
Kinv = inv(K);
P_cam = Kinv * pix .* d_vec.;

% Transform into base frame with T_base_cam (4x4)
T_base_cam = eye(4, "single");
P_cam_h = [P_cam; ones(1, size(P_cam,2), "single")];
P_base_h = T_base_cam * P_cam_h;
P_base = P_base_h(1:3,:).';

% P_base is an N-by-3 array of 3D points for the target object
      
