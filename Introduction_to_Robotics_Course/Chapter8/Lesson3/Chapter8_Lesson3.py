import cv2
import numpy as np

img = cv2.imread("scene.png", cv2.IMREAD_GRAYSCALE)

# --- Edges (Sobel magnitude) ---
Ix = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
Iy = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)
G  = np.sqrt(Ix**2 + Iy**2)
edges = (G > 50).astype(np.uint8) * 255

# --- Corners (Harris) ---
harris = cv2.cornerHarris(np.float32(img), blockSize=2, ksize=3, k=0.04)
corners = np.argwhere(harris > 0.01 * harris.max())

# --- Blobs (DoG / SIFT-like detector, but only detection) ---
blob_detector = cv2.SimpleBlobDetector_create()
keypoints = blob_detector.detect(img)

print("corners:", len(corners), "blobs:", len(keypoints))
