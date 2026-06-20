import numpy as np
import open3d as o3d

def rigid_transform_svd(P, Q):
    """
    Solve for R, t that minimize sum ||R p_i + t - q_i||^2
    P, Q: (N, 3) numpy arrays of corresponding points.
    """
    assert P.shape == Q.shape
    N = P.shape[0]

    # Compute centroids
    p_bar = P.mean(axis=0)
    q_bar = Q.mean(axis=0)

    # Center the points
    P_centered = P - p_bar
    Q_centered = Q - q_bar

    # Cross-covariance matrix H
    H = P_centered.T @ Q_centered

    # SVD of H
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Handle possible reflection
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1.0
        R = Vt.T @ U.T

    t = q_bar - R @ p_bar
    return R, t

# Example: using Open3D for ICP with point-to-point metric
source = o3d.io.read_point_cloud("model_cloud.pcd")
target = o3d.io.read_point_cloud("scene_cloud.pcd")

# Downsample for speed
source_ds = source.voxel_down_sample(voxel_size=0.005)
target_ds = target.voxel_down_sample(voxel_size=0.005)

# Initial guess (identity)
init_transform = np.eye(4)

threshold = 0.02  # max correspondence distance in meters
icp_result = o3d.pipelines.registration.registration_icp(
    source_ds,
    target_ds,
    threshold,
    init_transform,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

print("ICP fitness:", icp_result.fitness)
print("ICP inlier_rmse:", icp_result.inlier_rmse)
print("Estimated transform:\n", icp_result.transformation)
      
