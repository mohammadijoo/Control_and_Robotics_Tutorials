import open3d as o3d

# Assume pcd_t and pcd_tp1 are open3d.geometry.PointCloud objects
# obtained from two RGB-D frames via open3d.geometry.RGBDImage.create_from_...

threshold = 0.02  # correspondence distance
trans_init = np.eye(4)

reg = o3d.pipelines.registration.registration_icp(
    pcd_t, pcd_tp1, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

T = reg.transformation  # 4x4 rigid transform from time t to t+1

# Scene flow approximation: v(x) ≈ (T * x - x) / dt for each point x
points = np.asarray(pcd_t.points)
ones = np.ones((points.shape[0], 1))
hom = np.hstack((points, ones))
moved = (T @ hom.T).T[:, :3]
flow = (moved - points) / kf.dt  # approximate 3D velocity per point
      
