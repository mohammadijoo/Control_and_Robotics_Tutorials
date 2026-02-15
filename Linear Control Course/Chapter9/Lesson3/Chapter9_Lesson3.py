import numpy as np

def centroid_and_angles(poles, zeros):
    """
    Compute centroid and asymptote angles for root locus of
    G(s) = K * N(s) / D(s).
    poles, zeros: 1D arrays of complex numbers (open-loop poles/zeros).
    """
    poles = np.asarray(poles, dtype=complex)
    zeros = np.asarray(zeros, dtype=complex)
    npoles = len(poles)
    nzeros = len(zeros)

    if nzeros == 0:
        zeros = np.array([], dtype=complex)

    qa = npoles - nzeros  # number of asymptotes
    if qa <= 0:
        raise ValueError("No asymptotes: npoles <= nzeros")

    sigma_a = (np.sum(poles) - np.sum(zeros)) / qa
    angles = [(2*k + 1) * np.pi / qa for k in range(qa)]
    return sigma_a, angles

def poly_from_roots(roots):
    """Return monic polynomial coefficients with given roots."""
    roots = np.asarray(roots, dtype=complex)
    if len(roots) == 0:
        return np.array([1.0])
    return np.poly(roots)  # highest degree first

def breakaway_candidates(poles, zeros, tol_im=1e-6):
    """
    Compute candidate breakaway/break-in points by solving
    N(s) D'(s) - D(s) N'(s) = 0.
    """
    D = poly_from_roots(poles)
    N = poly_from_roots(zeros)

    Dp = np.polyder(D)
    Np = np.polyder(N)

    # P(s) = N(s) D'(s) - D(s) N'(s)
    P = np.polysub(np.polymul(N, Dp), np.polymul(D, Np))
    roots_P = np.roots(P)

    # keep approximately real roots
    real_points = []
    for r in roots_P:
        if abs(r.imag) < tol_im:
            real_points.append(r.real)
    return np.array(sorted(real_points))

def is_on_real_axis_locus(x, poles, zeros, tol=1e-6):
    """
    Check if a real point x lies on the real-axis segment of the root locus
    using the 'odd-number of poles and zeros to the right' rule.
    """
    count = 0
    for p in poles:
        if abs(p.imag) < tol and p.real > x:
            count += 1
    for z in zeros:
        if abs(z.imag) < tol and z.real > x:
            count += 1
    return (count % 2) == 1

def K_of_s(s, poles, zeros):
    """Compute gain K(s) = -D(s)/N(s) for scalar s."""
    D = poly_from_roots(poles)
    N = poly_from_roots(zeros)
    Ds = np.polyval(D, s)
    Ns = np.polyval(N, s)
    return -Ds / Ns

# Example: DC motor position loop (simplified)
poles = [-2.0, -5.0, -20.0]  # open-loop poles
zeros = []                   # assume no open-loop zeros

sigma_a, angles = centroid_and_angles(poles, zeros)
print("Centroid sigma_a =", sigma_a)
print("Asymptote angles (deg) =", [a * 180/np.pi for a in angles])

cands = breakaway_candidates(poles, zeros)
print("Raw real candidates:", cands)

valid_break_points = []
for x in cands:
    if is_on_real_axis_locus(x, poles, zeros):
        Kx = K_of_s(x, poles, zeros)
        if np.isreal(Kx) and Kx > 0:
            valid_break_points.append((x, float(np.real(Kx))))

print("Valid breakaway/break-in points (x, K):")
for x, k in valid_break_points:
    print(f"  s = {x:.4f}, K = {k:.4f}")
