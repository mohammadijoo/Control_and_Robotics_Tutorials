# Chapter16_Lesson5.py
"""
Discrete-time stability and time response demo for System Dynamics (Chapter 16, Lesson 5)

Topics:
- Schur stability (eigenvalues inside unit circle)
- Natural/forced response
- Discrete-time Lyapunov equation (iterative solver)
- Step response metrics (rise time, settling time, overshoot)
"""
import numpy as np

def spectral_radius(A: np.ndarray) -> float:
    vals = np.linalg.eigvals(A)
    return float(np.max(np.abs(vals)))

def is_schur_stable(A: np.ndarray, tol: float = 1e-12) -> bool:
    vals = np.linalg.eigvals(A)
    return bool(np.all(np.abs(vals) < 1.0 - tol))

def simulate_state_space(A, B, C, D, u, x0):
    """
    x[k+1] = A x[k] + B u[k]
    y[k]   = C x[k] + D u[k]
    """
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float).reshape((-1, 1))
    C = np.array(C, dtype=float).reshape((1, -1))
    D = np.array(D, dtype=float).reshape((1, 1))
    u = np.asarray(u, dtype=float).reshape((-1,))
    x = np.array(x0, dtype=float).reshape((-1, 1))

    n = len(u)
    xs = [x.copy()]
    ys = []
    for k in range(n):
        yk = C @ x + D * u[k]
        ys.append(float(yk[0, 0]))
        x = A @ x + B * u[k]
        xs.append(x.copy())
    X = np.hstack(xs).T
    y = np.array(ys)
    return X, y

def dlyap_iterative(A, Q, max_iter=20000, tol=1e-12):
    """
    Solve P = A^T P A + Q by summation:
    P = sum_{k=0}^{inf} (A^k)^T Q A^k, valid for Schur-stable A.
    """
    A = np.array(A, dtype=float)
    Q = np.array(Q, dtype=float)
    n = A.shape[0]
    P = np.zeros((n, n))
    Ak = np.eye(n)
    for _ in range(max_iter):
        term = Ak.T @ Q @ Ak
        P_next = P + term
        if np.linalg.norm(term, ord='fro') < tol:
            return 0.5 * (P_next + P_next.T)
        P = P_next
        Ak = Ak @ A
    raise RuntimeError("dlyap_iterative did not converge. Check Schur stability or increase max_iter.")

def step_metrics(y, final_value=None, tol=0.02):
    y = np.asarray(y, dtype=float)
    N = y.size
    if final_value is None:
        final_value = float(np.mean(y[max(0, N//2):]))
    if abs(final_value) < 1e-12:
        return {"final": final_value, "rise_index": None, "settling_index": None, "overshoot_pct": 0.0}

    # rise time index (10% to 90%)
    y10 = 0.1 * final_value
    y90 = 0.9 * final_value
    idx10 = next((i for i, yi in enumerate(y) if yi >= y10), None)
    idx90 = next((i for i, yi in enumerate(y) if yi >= y90), None)
    rise_index = None if idx10 is None or idx90 is None else idx90 - idx10

    band = tol * abs(final_value)
    settling_index = None
    for i in range(N):
        if np.all(np.abs(y[i:] - final_value) <= band):
            settling_index = i
            break

    overshoot = (np.max(y) - final_value) / abs(final_value) * 100.0
    return {
        "final": final_value,
        "rise_index": rise_index,
        "settling_index": settling_index,
        "overshoot_pct": overshoot,
    }

def main():
    # A stable second-order discrete-time model
    # Poles are approximately 0.82*exp(±j0.28)
    A = np.array([[1.5770, -0.6724],
                  [1.0000,  0.0000]])
    B = np.array([[1.0],
                  [0.0]])
    C = np.array([[0.0676, 0.0604]])
    D = np.array([[0.0]])

    print("A =\n", A)
    eigA = np.linalg.eigvals(A)
    print("Eigenvalues(A) =", eigA)
    print("Spectral radius =", spectral_radius(A))
    print("Schur stable?   =", is_schur_stable(A))

    # Lyapunov certificate
    Q = np.eye(2)
    P = dlyap_iterative(A, Q)
    M = A.T @ P @ A - P
    print("\nDiscrete Lyapunov matrix P (Q=I):\n", P)
    print("Check A^T P A - P (should be -Q):\n", M)

    # Time response to step input
    N = 80
    u = np.ones(N)
    x0 = np.array([0.0, 0.0])
    X, y = simulate_state_space(A, B, C, D, u, x0)

    # Natural response from nonzero initial condition
    Xn, yn = simulate_state_space(A, B, C, D, np.zeros(N), np.array([1.0, -0.4]))

    metrics = step_metrics(y, final_value=1.0)  # DC gain chosen ~1
    print("\nStep-response metrics (sample counts):")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    print("\nFirst 12 samples of unit-step response:")
    print(np.array2string(y[:12], precision=5, separator=", "))

    print("\nFirst 12 samples of natural response (u=0, x0=[1,-0.4]):")
    print(np.array2string(yn[:12], precision=5, separator=", "))

if __name__ == "__main__":
    main()
