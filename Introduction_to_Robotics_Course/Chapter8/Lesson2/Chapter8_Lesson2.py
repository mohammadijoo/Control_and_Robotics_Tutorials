import cv2
import numpy as np
import glob

# Checkerboard settings
pattern_size = (9, 6)  # inner corners (cols, rows)
square_size = 0.025    # meters

# Prepare known 3D points on plane Z=0
objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D target points in reference coords
imgpoints = []  # detected 2D points

images = glob.glob("calib_images/*.png")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, pattern_size)

    if ok:
        # refine corners
        corners = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("RMS reprojection error:", ret)
print("Intrinsic matrix K:\n", K)
print("Distortion coefficients:\n", dist.ravel())

# Example: get extrinsic for first view
R0, _ = cv2.Rodrigues(rvecs[0])
t0 = tvecs[0]
print("Extrinsic (first image):")
print("R0=\n", R0)
print("t0=\n", t0)
