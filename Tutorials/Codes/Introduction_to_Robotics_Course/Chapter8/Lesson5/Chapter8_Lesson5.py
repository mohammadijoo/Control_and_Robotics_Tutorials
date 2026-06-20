import cv2
import numpy as np

# (1) Acquire
img = cv2.imread("scene.png", cv2.IMREAD_GRAYSCALE)

# (2) Preprocess: Gaussian smoothing
img_s = cv2.GaussianBlur(img, (5, 5), 1.2)

# (3) Feature extraction: Shi-Tomasi corners (structure tensor based)
corners = cv2.goodFeaturesToTrack(img_s, maxCorners=200, qualityLevel=0.01, minDistance=8)
corners = np.squeeze(corners)  # shape: (N,2)

# (4) Deterministic fusion example:
# Suppose two sensors give noisy position estimates x1, x2 with covariances S1, S2
x1 = np.array([1.2, 0.5])
x2 = np.array([1.0, 0.9])
S1 = np.diag([0.04, 0.09])
S2 = np.diag([0.16, 0.04])

W1 = np.linalg.inv(S1)
W2 = np.linalg.inv(S2)
xf = np.linalg.inv(W1 + W2) @ (W1 @ x1 + W2 @ x2)

print("Fused estimate:", xf)
