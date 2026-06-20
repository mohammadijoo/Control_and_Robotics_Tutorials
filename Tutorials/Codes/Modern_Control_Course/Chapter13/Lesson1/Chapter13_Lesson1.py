# Chapter13_Lesson1.py
# Intuitive observability from output measurements for continuous-time LTI systems.
# Requirements: numpy, scipy. Optional: python-control for comparison.

import numpy as np
from numpy.linalg import matrix_rank, pinv, norm
from scipy.linalg import expm


def observability_matrix(A: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Build O_n = [C; C A; ...; C A^(n-1)]."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float)
    n = A.shape[0]
    blocks = []
    Ak = np.eye(n)
    for _ in range(n):
        blocks.append(C @ Ak)
        Ak = Ak @ A
    return np.vstack(blocks)


def output_from_initial_state(A, C, x0, times):
    """Zero-input output y(t)=C exp(A t) x0 for t0=0."""
    return np.array([(C @ expm(A * t) @ x0).ravel() for t in times])


def derivative_stack_measurement(A, C, x0):
    """Stack y(0), y_dot(0), ..., y^(n-1)(0) = O_n x0."""
    O = observability_matrix(A, C)
    return O @ x0


def reconstruct_x0_from_derivatives(A, C, derivative_vector):
    """Least-squares reconstruction of x0 from stacked derivatives."""
    O = observability_matrix(A, C)
    return pinv(O) @ derivative_vector


def report_system(name, A, C, x_true):
    O = observability_matrix(A, C)
    print(f"\n{name}")
    print("A =\n", A)
    print("C =\n", C)
    print("O_n =\n", O)
    print("rank(O_n) =", matrix_rank(O), "out of n =", A.shape[0])

    z = derivative_stack_measurement(A, C, x_true)
    x_hat = reconstruct_x0_from_derivatives(A, C, z)
    print("true x0 =", x_true.ravel())
    print("stacked derivatives =", z.ravel())
    print("least-squares reconstructed x0 =", x_hat.ravel())
    print("reconstruction error norm =", norm(x_hat - x_true))


if __name__ == "__main__":
    # Example 1: measuring position reveals velocity through the derivative of position.
    A1 = np.array([[0.0, 1.0],
                   [-2.0, -3.0]])
    C1 = np.array([[1.0, 0.0]])
    x01 = np.array([[1.0], [2.0]])
    report_system("Example 1: position sensor, observable second-order system", A1, C1, x01)

    # Show output traces for two different initial states.
    times = np.linspace(0.0, 5.0, 101)
    y1 = output_from_initial_state(A1, C1, np.array([[1.0], [2.0]]), times)
    y2 = output_from_initial_state(A1, C1, np.array([[1.0], [-1.0]]), times)
    print("\nFirst five zero-input outputs for x0=[1,2]^T:", y1[:5, 0])
    print("First five zero-input outputs for x0=[1,-1]^T:", y2[:5, 0])

    # Example 2: diagonal dynamics with a sensor that sees only the first coordinate.
    # The second state never affects the output, so it is an unobservable direction.
    A2 = np.array([[-1.0, 0.0],
                   [0.0, -2.0]])
    C2 = np.array([[1.0, 0.0]])
    x02 = np.array([[3.0], [4.0]])
    report_system("Example 2: second mode invisible, unobservable", A2, C2, x02)

    # If desired and installed:
    # import control
    # print(control.obsv(A1, C1))
