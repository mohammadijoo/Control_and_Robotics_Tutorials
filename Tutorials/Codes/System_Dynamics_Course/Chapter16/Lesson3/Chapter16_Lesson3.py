# Chapter16_Lesson3.py
import numpy as np

def impulse_response(b, a, N=20):
    x = np.zeros(N); x[0] = 1.0
    y = np.zeros(N)
    for n in range(N):
        y[n] += sum(b[k] * x[n-k] for k in range(len(b)) if n-k >= 0)
        y[n] -= sum(a[k] * y[n-k] for k in range(1, len(a)) if n-k >= 0)
    return y / a[0]

def H_ejw(b, a, w):
    zinv = np.exp(-1j * w)
    num = sum(b[k] * zinv**k for k in range(len(b)))
    den = sum(a[k] * zinv**k for k in range(len(a)))
    return num / den

if __name__ == "__main__":
    # H(z) = (0.2 + 0.1 z^-1) / (1 - 1.5 z^-1 + 0.56 z^-2)
    b = np.array([0.2, 0.1])
    a = np.array([1.0, -1.5, 0.56])

    h = impulse_response(b, a, 20)
    print("h[n] (first 10 samples):", np.round(h[:10], 6))

    ws = np.linspace(0, np.pi, 5)
    for w in ws:
        H = H_ejw(b, a, w)
        print(f"w={w:.3f}, |H|={abs(H):.6f}, phase={np.angle(H):.6f}")

    poles = np.roots(a)
    zeros = np.roots(np.array([0.2, 0.1]))
    print("poles:", poles, "stable:", np.all(np.abs(poles) < 1))
    print("zeros:", zeros)
