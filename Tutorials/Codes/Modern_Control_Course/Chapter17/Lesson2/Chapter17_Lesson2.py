# Chapter17_Lesson2.py
# CCF-OCF duality for a strictly proper SISO transfer function.
# Requires: numpy

import numpy as np


def ctrb(A, B):
    n = A.shape[0]
    blocks, Ak = [], np.eye(n)
    for _ in range(n):
        blocks.append(Ak @ B)
        Ak = Ak @ A
    return np.hstack(blocks)


def obsv(A, C):
    n = A.shape[0]
    rows, Ak = [], np.eye(n)
    for _ in range(n):
        rows.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(rows)


def ccf_ocf(num_desc, den_desc):
    # den_desc = [1, a_{n-1}, ..., a0]
    # num_desc = [b_{n-1}, ..., b0], padded if necessary.
    den_desc = np.asarray(den_desc, dtype=float)
    num_desc = np.asarray(num_desc, dtype=float)

    if abs(den_desc[0] - 1.0) > 1e-12:
        num_desc = num_desc / den_desc[0]
        den_desc = den_desc / den_desc[0]

    n = len(den_desc) - 1
    if len(num_desc) > n:
        raise ValueError("This script assumes a strictly proper transfer function.")

    a = den_desc[1:][::-1]                         # [a0,...,a_{n-1}]
    b = np.pad(num_desc, (n-len(num_desc), 0))[::-1] # [b0,...,b_{n-1}]

    Ac = np.zeros((n, n))
    Ac[:-1, 1:] = np.eye(n - 1)
    Ac[-1, :] = -a

    Bc = np.zeros((n, 1))
    Bc[-1, 0] = 1.0

    Cc = b.reshape(1, n)
    D = np.array([[0.0]])

    Ao = Ac.T
    Bo = Cc.T
    Co = Bc.T

    return Ac, Bc, Cc, D, Ao, Bo, Co, D.copy()


def H(A, B, C, D, s):
    n = A.shape[0]
    return (C @ np.linalg.solve(s*np.eye(n) - A, B) + D)[0, 0]


if __name__ == "__main__":
    # G(s) = (2s^2 + 5s + 3)/(s^3 + 4s^2 + 6s + 4)
    num = [2.0, 5.0, 3.0]
    den = [1.0, 4.0, 6.0, 4.0]

    Ac, Bc, Cc, Dc, Ao, Bo, Co, Do = ccf_ocf(num, den)

    print("Ac =\n", Ac)
    print("Bc =\n", Bc)
    print("Cc =\n", Cc)
    print("Ao = Ac.T =\n", Ao)
    print("Bo = Cc.T =\n", Bo)
    print("Co = Bc.T =\n", Co)

    Qc = ctrb(Ac, Bc)
    Oo = obsv(Ao, Co)

    print("rank ctrb(Ac,Bc) =", np.linalg.matrix_rank(Qc))
    print("rank obsv(Ao,Co) =", np.linalg.matrix_rank(Oo))
    print("||Oo - Qc.T||_F  =", np.linalg.norm(Oo - Qc.T))

    for s in [0.5, 1.0, 2.0, 3.0]:
        hc = H(Ac, Bc, Cc, Dc, s)
        ho = H(Ao, Bo, Co, Do, s)
        print(f"s={s:.1f}: H_CCF={hc:.10f}, H_OCF={ho:.10f}, error={abs(hc-ho):.2e}")
