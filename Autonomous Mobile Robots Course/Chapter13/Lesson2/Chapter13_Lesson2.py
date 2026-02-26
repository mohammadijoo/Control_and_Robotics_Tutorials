# Chapter13_Lesson2.py
"""
Autonomous Mobile Robots — Chapter 13, Lesson 2
Feature-Based vs Direct Methods (minimal runnable demo)

Dependencies:
  pip install opencv-python numpy

Usage:
  python Chapter13_Lesson2.py --img1 path/to/frame1.png --img2 path/to/frame2.png
  (Optionally provide camera intrinsics via --fx --fy --cx --cy)
"""
import argparse
import numpy as np
import cv2

def load_gray(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return img

def feature_based_relative_pose(I1, I2, K, nfeatures=2000):
    """ORB + matching + Essential matrix (calibrated) + recoverPose.
    Returns (R, t_unit, inlier_mask, pts1, pts2) where t has unit norm (scale ambiguous for monocular).
    """
    orb = cv2.ORB_create(nfeatures=nfeatures)
    k1, d1 = orb.detectAndCompute(I1, None)
    k2, d2 = orb.detectAndCompute(I2, None)
    if d1 is None or d2 is None or len(k1) < 8 or len(k2) < 8:
        raise RuntimeError("Not enough features/descriptors to estimate pose.")

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(d1, d2, k=2)

    # Lowe ratio test
    good = []
    for m, n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    if len(good) < 8:
        raise RuntimeError("Not enough good matches after ratio test.")

    pts1 = np.float32([k1[m.queryIdx].pt for m in good])
    pts2 = np.float32([k2[m.trainIdx].pt for m in good])

    # Robust Essential matrix with RANSAC
    E, inliers = cv2.findEssentialMat(
        pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0
    )
    if E is None:
        raise RuntimeError("findEssentialMat failed.")

    # recoverPose expects inliers mask
    inliers = inliers.astype(bool).ravel()
    pts1_in = pts1[inliers]
    pts2_in = pts2[inliers]
    if len(pts1_in) < 8:
        raise RuntimeError("Not enough inliers to recover pose.")

    _, R, t, _ = cv2.recoverPose(E, pts1_in, pts2_in, K)
    t = t.ravel()
    t_unit = t / (np.linalg.norm(t) + 1e-12)
    return R, t_unit, inliers, pts1, pts2

def robust_huber_weights(r, delta):
    """Huber weights for residual vector r (1D)."""
    a = np.abs(r)
    w = np.ones_like(r, dtype=np.float32)
    mask = a > delta
    w[mask] = (delta / (a[mask] + 1e-12)).astype(np.float32)
    return w

def direct_align_translation(I_ref, I_cur, levels=4, iters=10, huber_delta=5.0):
    """Coarse-to-fine direct alignment for 2D translation (u,v) in pixels.
    Minimizes sum rho(I_cur(x+u,y+v) - I_ref(x,y)).
    """
    # Build pyramids
    pyr_ref = [I_ref.astype(np.float32)]
    pyr_cur = [I_cur.astype(np.float32)]
    for _ in range(1, levels):
        pyr_ref.append(cv2.pyrDown(pyr_ref[-1]))
        pyr_cur.append(cv2.pyrDown(pyr_cur[-1]))

    u, v = 0.0, 0.0  # translation at current pyramid scale
    for lvl in reversed(range(levels)):
        R = pyr_ref[lvl]
        C = pyr_cur[lvl]
        # scale translation to this level
        scale = 2.0 ** lvl
        u_lvl = u / scale
        v_lvl = v / scale

        for _ in range(iters):
            # Warp current image by (u_lvl, v_lvl)
            M = np.array([[1.0, 0.0, u_lvl],
                          [0.0, 1.0, v_lvl]], dtype=np.float32)
            Cw = cv2.warpAffine(C, M, (R.shape[1], R.shape[0]), flags=cv2.INTER_LINEAR)

            # Compute residual and gradients
            r = (Cw - R)

            # Use gradients of warped current image (forward-additive LK)
            Ix = cv2.Sobel(Cw, cv2.CV_32F, 1, 0, ksize=3)
            Iy = cv2.Sobel(Cw, cv2.CV_32F, 0, 1, ksize=3)

            # Exclude a border to avoid interpolation artifacts
            b = 10
            r0 = r[b:-b, b:-b].reshape(-1)
            gx = Ix[b:-b, b:-b].reshape(-1)
            gy = Iy[b:-b, b:-b].reshape(-1)

            # Reject near-zero gradient pixels
            g2 = gx*gx + gy*gy
            mask = g2 > 1e-4
            r0 = r0[mask]
            gx = gx[mask]
            gy = gy[mask]
            if r0.size < 200:
                break

            w = robust_huber_weights(r0, huber_delta)
            # Normal equations for translation: A = [gx gy], solve (A^T W A) d = -A^T W r
            A11 = float(np.sum(w * gx * gx))
            A12 = float(np.sum(w * gx * gy))
            A22 = float(np.sum(w * gy * gy))
            b1  = float(np.sum(w * gx * r0))
            b2  = float(np.sum(w * gy * r0))

            H = np.array([[A11, A12],
                          [A12, A22]], dtype=np.float64)
            g = np.array([b1, b2], dtype=np.float64)

            # Solve with damping if ill-conditioned
            lam = 1e-3 * (H[0,0] + H[1,1] + 1e-12)
            H_damped = H + lam * np.eye(2)

            try:
                duv = -np.linalg.solve(H_damped, g)
            except np.linalg.LinAlgError:
                break

            u_lvl += float(duv[0])
            v_lvl += float(duv[1])

            if np.linalg.norm(duv) < 1e-3:
                break

        # bring back to full-res units
        u = u_lvl * scale
        v = v_lvl * scale

    return u, v

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img1", required=True, help="Path to reference image (t-1).")
    ap.add_argument("--img2", required=True, help="Path to current image (t).")
    ap.add_argument("--fx", type=float, default=525.0)
    ap.add_argument("--fy", type=float, default=525.0)
    ap.add_argument("--cx", type=float, default=319.5)
    ap.add_argument("--cy", type=float, default=239.5)
    ap.add_argument("--no_vis", action="store_true", help="Disable visualization windows.")
    args = ap.parse_args()

    I1 = load_gray(args.img1)
    I2 = load_gray(args.img2)

    K = np.array([[args.fx, 0.0, args.cx],
                  [0.0, args.fy, args.cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)

    print("=== Feature-based (ORB + Essential + recoverPose) ===")
    R, t_unit, inliers, pts1, pts2 = feature_based_relative_pose(I1, I2, K)
    print("R =\n", R)
    print("t (unit norm) =", t_unit, "(monocular scale unknown)")

    print("\n=== Direct (translation-only photometric alignment) ===")
    u, v = direct_align_translation(I1, I2, levels=4, iters=15, huber_delta=5.0)
    print(f"Estimated pixel translation: u={u:.3f}, v={v:.3f}")

    if not args.no_vis:
        # Draw inlier matches
        orb = cv2.ORB_create(nfeatures=2000)
        k1, d1 = orb.detectAndCompute(I1, None)
        k2, d2 = orb.detectAndCompute(I2, None)
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        matches = bf.knnMatch(d1, d2, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good.append(m)
        # Mask by inliers computed earlier (aligned with 'good' list)
        inlier_list = [m for m, ok in zip(good, inliers.tolist()) if ok]
        vis = cv2.drawMatches(I1, k1, I2, k2, inlier_list[:60], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("Inlier matches (top 60)", vis)

        # Visualize direct alignment by warping I2 back toward I1
        M = np.array([[1.0, 0.0, -u],
                      [0.0, 1.0, -v]], dtype=np.float32)
        I2w = cv2.warpAffine(I2, M, (I1.shape[1], I1.shape[0]))
        diff = cv2.absdiff(I1, I2w)
        cv2.imshow("Direct alignment: abs(I1 - warp(I2))", diff)

        cv2.waitKey(0)

if __name__ == "__main__":
    main()
