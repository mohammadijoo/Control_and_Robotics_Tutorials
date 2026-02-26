#!/usr/bin/env python3
# Chapter13_Lesson1.py
"""
Autonomous Mobile Robots (Control Engineering)
Chapter 13 - Visual and Visual–Inertial SLAM (AMR Focus)
Lesson 1  - Visual Odometry for Mobile Robots

Minimal feature-based monocular VO:
  - ORB keypoints + descriptor matching
  - Essential matrix with RANSAC
  - Relative pose via recoverPose
  - Pose chaining to get a trajectory (up to unknown global scale)

AMR note:
  - Monocular VO has scale ambiguity. This script optionally accepts per-frame
    scale factors from wheel odometry (or any external source) to metrify the trajectory.

Dependencies:
  pip install numpy opencv-python

Usage examples:
  python Chapter13_Lesson1.py --images path/to/frames --K path/to/K.json
  python Chapter13_Lesson1.py --images path/to/frames --scale_csv odom_scales.csv
"""

from __future__ import annotations
import argparse
import glob
import json
import os
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import cv2


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)


def load_intrinsics(path: Optional[str]) -> Intrinsics:
    # If no intrinsics file is provided, use a common VGA-like placeholder.
    if path is None:
        return Intrinsics(fx=525.0, fy=525.0, cx=319.5, cy=239.5)

    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    return Intrinsics(fx=float(d["fx"]), fy=float(d["fy"]), cx=float(d["cx"]), cy=float(d["cy"]))


def load_scales(path: Optional[str]) -> Optional[List[float]]:
    """
    Load per-step scales (meters) from a CSV with one number per line.
    scale[i] multiplies the translation t estimated between frame i and i+1.
    """
    if path is None:
        return None
    scales: List[float] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            scales.append(float(line.split(",")[0]))
    return scales


def list_images(folder: str) -> List[str]:
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp")
    files: List[str] = []
    for e in exts:
        files.extend(glob.glob(os.path.join(folder, e)))
    files.sort()
    if len(files) < 2:
        raise ValueError("Need at least 2 images in --images folder.")
    return files


def robust_match(des1: np.ndarray, des2: np.ndarray, ratio: float = 0.75) -> List[cv2.DMatch]:
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = bf.knnMatch(des1, des2, k=2)
    good: List[cv2.DMatch] = []
    for m, n in knn:
        if m.distance < ratio * n.distance:
            good.append(m)
    return good


def enforce_planar_rotation(R: np.ndarray) -> np.ndarray:
    """
    Optional AMR heuristic: project rotation onto pure yaw (zero roll/pitch).
    This is NOT a substitute for proper estimation; it's a pragmatic constraint.
    """
    # yaw from rotation matrix
    yaw = np.arctan2(R[1, 0], R[0, 0])
    cy = float(np.cos(yaw))
    sy = float(np.sin(yaw))
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
    return Rz


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="Folder containing a time-ordered image sequence")
    ap.add_argument("--K", default=None, help="Camera intrinsics JSON: {fx,fy,cx,cy}")
    ap.add_argument("--scale_csv", default=None, help="CSV with per-step scale (meters), one per line")
    ap.add_argument("--planar", action="store_true", help="Enforce planar yaw-only rotation (AMR heuristic)")
    ap.add_argument("--max_frames", type=int, default=0, help="Process only first N frames (0 = all)")
    args = ap.parse_args()

    intr = load_intrinsics(args.K)
    K = intr.K()

    scales = load_scales(args.scale_csv)

    files = list_images(args.images)
    if args.max_frames and args.max_frames > 1:
        files = files[: args.max_frames]

    orb = cv2.ORB_create(nfeatures=2000)

    img0 = cv2.imread(files[0], cv2.IMREAD_GRAYSCALE)
    if img0 is None:
        raise RuntimeError(f"Cannot read: {files[0]}")
    kp0, des0 = orb.detectAndCompute(img0, None)

    # World frame = camera frame at t=0
    T_cw = np.eye(4, dtype=np.float64)  # transform world -> current camera
    traj = []  # camera centers in world

    def camera_center_world(T_cw_: np.ndarray) -> np.ndarray:
        R_cw = T_cw_[:3, :3]
        t_cw = T_cw_[:3, 3]
        C_w = -R_cw.T @ t_cw
        return C_w

    traj.append(camera_center_world(T_cw))

    for i in range(1, len(files)):
        img1 = cv2.imread(files[i], cv2.IMREAD_GRAYSCALE)
        if img1 is None:
            raise RuntimeError(f"Cannot read: {files[i]}")

        kp1, des1 = orb.detectAndCompute(img1, None)
        if des0 is None or des1 is None or len(kp0) < 20 or len(kp1) < 20:
            print(f"[WARN] Not enough features at frame {i}. Skipping.")
            kp0, des0 = kp1, des1
            continue

        matches = robust_match(des0, des1, ratio=0.75)
        if len(matches) < 20:
            print(f"[WARN] Not enough matches at frame {i}. Skipping.")
            kp0, des0 = kp1, des1
            continue

        pts0 = np.float32([kp0[m.queryIdx].pt for m in matches])
        pts1 = np.float32([kp1[m.trainIdx].pt for m in matches])

        E, inliers = cv2.findEssentialMat(
            pts0, pts1, K, method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        if E is None:
            print(f"[WARN] Essential matrix failed at frame {i}. Skipping.")
            kp0, des0 = kp1, des1
            continue

        _, R, t, _ = cv2.recoverPose(E, pts0, pts1, K, mask=inliers)
        if args.planar:
            R = enforce_planar_rotation(R)

        # scale (meters). If not provided, keep unit scale.
        s = 1.0
        if scales is not None:
            if (i - 1) < len(scales):
                s = float(scales[i - 1])
            else:
                s = float(scales[-1])

        T_12 = np.eye(4, dtype=np.float64)
        T_12[:3, :3] = R
        T_12[:3, 3] = (s * t.reshape(3))

        T_cw = T_12 @ T_cw
        traj.append(camera_center_world(T_cw))

        kp0, des0 = kp1, des1

        if i % 10 == 0 or i == len(files) - 1:
            C = traj[-1]
            print(f"[INFO] frame {i:4d}/{len(files)-1}: C_w = {C[0]: .3f}, {C[1]: .3f}, {C[2]: .3f}")

    traj = np.array(traj, dtype=np.float64)
    out = os.path.join(args.images, "vo_trajectory_xyz.csv")
    np.savetxt(out, traj, delimiter=",", header="x,y,z", comments="")
    print("[DONE] trajectory saved to:", out)


if __name__ == "__main__":
    main()
