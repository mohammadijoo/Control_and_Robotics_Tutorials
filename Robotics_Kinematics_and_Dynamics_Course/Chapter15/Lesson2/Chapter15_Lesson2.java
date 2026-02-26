public class CircleConstrainedDynamics {

    static class State {
        double x, y;
        double xdot, ydot;
    }

    static class Result {
        double xddot, yddot;
        double lambda;
    }

    public static Result step(State s, double m, double g) {
        // M = m * I
        double[][] M = {
            {m, 0.0},
            {0.0, m}
        };

        // J = [2x 2y]
        double[] J = {2.0 * s.x, 2.0 * s.y};

        // Jdot * qdot
        double Jdot_qdot = 2.0 * (s.xdot * s.xdot + s.ydot * s.ydot);
        double gamma = -Jdot_qdot;

        // Unconstrained generalized forces (gravity on y)
        double[] f = {0.0, -m * g};
        double[] tau = {0.0, 0.0};

        // Build augmented 3x3 system A * [xddot, yddot, lambda]^T = rhs
        double[][] A = new double[3][3];

        A[0][0] = M[0][0];
        A[0][1] = M[0][1];
        A[0][2] = -J[0];

        A[1][0] = M[1][0];
        A[1][1] = M[1][1];
        A[1][2] = -J[1];

        A[2][0] = J[0];
        A[2][1] = J[1];
        A[2][2] = 0.0;

        double[] rhs = new double[3];
        rhs[0] = tau[0] + f[0];
        rhs[1] = tau[1] + f[1];
        rhs[2] = gamma;

        // Solve 3x3 linear system via Gaussian elimination (omitted for brevity)
        double[] sol = solve3x3(A, rhs);

        Result r = new Result();
        r.xddot = sol[0];
        r.yddot = sol[1];
        r.lambda = sol[2];
        return r;
    }

    // Simple 3x3 solver (no pivoting)
    static double[] solve3x3(double[][] A, double[] b) {
        double[][] a = new double[3][4];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) a[i][j] = A[i][j];
            a[i][3] = b[i];
        }
        // Forward elimination
        for (int k = 0; k < 3; ++k) {
            double pivot = a[k][k];
            for (int j = k; j < 4; ++j) a[k][j] /= pivot;
            for (int i = k + 1; i < 3; ++i) {
                double factor = a[i][k];
                for (int j = k; j < 4; ++j) {
                    a[i][j] -= factor * a[k][j];
                }
            }
        }
        // Back substitution
        double[] x = new double[3];
        for (int i = 2; i >= 0; --i) {
            x[i] = a[i][3];
            for (int j = i + 1; j < 3; ++j) {
                x[i] -= a[i][j] * x[j];
            }
        }
        return x;
    }

    public static void main(String[] args) {
        State s = new State();
        s.x = 1.0; s.y = 0.0;
        s.xdot = 0.0; s.ydot = 1.0;

        Result r = step(s, 1.0, 9.81);
        System.out.println("xddot = " + r.xddot + ", yddot = " + r.yddot);
        System.out.println("lambda = " + r.lambda);
    }
}
      
