import numpy as np

def classify_ct_lti_from_den(den_coeffs, tol=1e-9):
    """
    Classify a continuous-time LTI system given the denominator coefficients
    of its transfer function G(s) = N(s) / D(s).

    den_coeffs: list or array [a_n, a_{n-1}, ..., a_0]
    tol: numerical tolerance for deciding whether Re(pole) is zero.
    """
    den_coeffs = np.asarray(den_coeffs, dtype=float)
    roots = np.roots(den_coeffs)  # poles of G(s)
    real_parts = roots.real

    max_real = np.max(real_parts)
    # Check for poles on the imaginary axis within tolerance
    on_imag_axis = np.abs(real_parts) <= tol

    if max_real < -tol:
        stability = "asymptotically stable"
    elif max_real > tol:
        stability = "unstable"
    else:
        # Some poles are near the imaginary axis, none clearly in right half-plane
        stability = "marginally stable (check multiplicities and resonance)"

    return roots, stability

if __name__ == "__main__":
    # Example: stable system G(s) = 1 / (s^2 + 3 s + 2)
    den = [1.0, 3.0, 2.0]
    poles, stability = classify_ct_lti_from_den(den)
    print("Poles:", poles)
    print("Classification:", stability)
