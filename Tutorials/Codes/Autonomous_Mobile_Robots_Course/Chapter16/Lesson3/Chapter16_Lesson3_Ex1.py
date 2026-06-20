# Chapter16_Lesson3_Ex1.py
# Exercise: compare isotropic vs anisotropic personal-space models

import numpy as np
import matplotlib.pyplot as plt

def rot2(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s],[s, c]])

def field(X, human_p, human_v, anisotropic=True):
    spd = np.linalg.norm(human_v)
    th = np.arctan2(human_v[1], human_v[0]) if spd > 1e-6 else 0.0
    R = rot2(th)
    Z = np.zeros(X.shape[:2])
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            x = X[i,j]
            r = x - human_p
            r_h = R.T @ r
            if anisotropic:
                sx = 1.4 if r_h[0] >= 0.0 else 0.9
                sy = 0.6
            else:
                sx = sy = 0.9
            q = (r_h[0]**2)/(sx**2) + (r_h[1]**2)/(sy**2)
            Z[i,j] = np.exp(-0.5*q)
    return Z

def main():
    human_p = np.array([0.0, 0.0])
    human_v = np.array([1.0, 0.0])

    xs = np.linspace(-3,3,200)
    ys = np.linspace(-3,3,200)
    X, Y = np.meshgrid(xs, ys)
    P = np.stack([X, Y], axis=-1)

    Z_iso = field(P, human_p, human_v, anisotropic=False)
    Z_an  = field(P, human_p, human_v, anisotropic=True)

    plt.figure(figsize=(12,5))
    plt.subplot(1,2,1); plt.contourf(X,Y,Z_iso, levels=20); plt.axis("equal"); plt.title("Isotropic comfort field")
    plt.subplot(1,2,2); plt.contourf(X,Y,Z_an,  levels=20); plt.axis("equal"); plt.title("Anisotropic comfort field")
    plt.show()

if __name__ == "__main__":
    main()
