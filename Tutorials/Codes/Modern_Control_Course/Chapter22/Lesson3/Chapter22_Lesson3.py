"""
Chapter22_Lesson3.py
State Feedback vs Output Feedback (Concept Only)

This script compares full-state feedback u = -K x + r with static output
feedback u = -F y + r for a small continuous-time LTI system.
"""

import numpy as np


def eig_summary(matrix: np.ndarray, name: str) -> None:
    eigvals = np.linalg.eigvals(matrix)
    print(f"{name} =")
    print(matrix)
    print(f"eigenvalues({name}) = {eigvals}")
    print(f"stable? {np.all(np.real(eigvals) < 0)}")
    print()


def main() -> None:
    # Continuous-time plant: x_dot = A x + B u, y = C x
    A = np.array([[0.0, 1.0], [-2.0, -0.4]])
    B = np.array([[0.0], [1.0]])
    C_full = np.eye(2)                 # full state is measured: y = x
    C_position = np.array([[1.0, 0.0]]) # only position is measured

    # Full-state feedback u = -K x.
    K = np.array([[4.0, 2.6]])
    A_state = A - B @ K

    # Static output feedback u = -F y = -F C x.
    # With position-only output, the effective gain has the restricted form [f, 0].
    F_position = np.array([[4.0]])
    A_output_position = A - B @ F_position @ C_position

    # If the full state is measured as y = x, static output feedback is equivalent
    # to state feedback by selecting F = K.
    F_full = K.copy()
    A_output_full = A - B @ F_full @ C_full

    eig_summary(A, "A_open_loop")
    eig_summary(A_state, "A_state_feedback")
    eig_summary(A_output_position, "A_static_output_feedback_position_only")
    eig_summary(A_output_full, "A_static_output_feedback_full_output")

    # Geometry: which state gains can be implemented as F C?
    # K_implementable iff K lies in the row space of C.
    rank_C = np.linalg.matrix_rank(C_position)
    rank_augmented = np.linalg.matrix_rank(np.vstack([C_position, K]))
    print("C_position rank:", rank_C)
    print("rank([C_position; K]):", rank_augmented)
    print("K implementable as F*C_position?", rank_augmented == rank_C)

    # Least-squares static output gain: minimize ||K - F C||_F.
    # For C = [1 0], the closest F C to K=[4 2.6] is [4 0].
    F_ls = K @ C_position.T @ np.linalg.pinv(C_position @ C_position.T)
    K_projected = F_ls @ C_position
    print("Least-squares F for output feedback:", F_ls)
    print("Projected effective gain F*C:", K_projected)


if __name__ == "__main__":
    main()
