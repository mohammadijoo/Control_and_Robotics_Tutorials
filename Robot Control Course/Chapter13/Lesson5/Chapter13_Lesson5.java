
public class SafetyFilter2D {

    private static final double GAMMA_JOINT = 5.0;
    private static final double GAMMA_WS = 5.0;
    private static final double R_OBS = 0.3;
    private static final double[] C_OBS = {0.5, 0.5};

    // Forward kinematics and Jacobian for planar 2-DOF arm
    public static double[] forwardKinematics(double[] q, double L1, double L2) {
        double q1 = q[0], q2 = q[1];
        double x = L1 * Math.cos(q1) + L2 * Math.cos(q1 + q2);
        double y = L1 * Math.sin(q1) + L2 * Math.sin(q1 + q2);
        return new double[]{x, y};
    }

    public static double[][] jacobian(double[] q, double L1, double L2) {
        double q1 = q[0], q2 = q[1];
        double s1 = Math.sin(q1), c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2), c12 = Math.cos(q1 + q2);
        double J11 = -L1 * s1 - L2 * s12;
        double J12 = -L2 * s12;
        double J21 =  L1 * c1 + L2 * c12;
        double J22 =  L2 * c12;
        return new double[][]{
                {J11, J12},
                {J21, J22}
        };
    }

    // Build inequalities A v <= b for given q
    public static void buildConstraints(double[] q,
                                        double[][] A,
                                        double[] b) {
        double[] qMin = {-Math.PI / 2.0, -Math.PI / 2.0};
        double[] qMax = { Math.PI / 2.0,  Math.PI / 2.0};

        int row = 0;
        // Joint limits
        for (int i = 0; i < 2; ++i) {
            double hMin = q[i] - qMin[i];
            // -e_i^T v <= gamma * hMin
            A[row][0] = (i == 0) ? -1.0 : 0.0;
            A[row][1] = (i == 1) ? -1.0 : 0.0;
            b[row] = GAMMA_JOINT * hMin;
            row++;

            double hMax = qMax[i] - q[i];
            // e_i^T v <= gamma * hMax
            A[row][0] = (i == 0) ? 1.0 : 0.0;
            A[row][1] = (i == 1) ? 1.0 : 0.0;
            b[row] = GAMMA_JOINT * hMax;
            row++;
        }

        // Workspace
        double L1 = 1.0, L2 = 1.0;
        double[] p = forwardKinematics(q, L1, L2);
        double[][] J = jacobian(q, L1, L2);
        double dx = p[0] - C_OBS[0];
        double dy = p[1] - C_OBS[1];
        double hWs = dx * dx + dy * dy - R_OBS * R_OBS;

        double a1 = -2.0 * (dx * J[0][0] + dy * J[1][0]);
        double a2 = -2.0 * (dx * J[0][1] + dy * J[1][1]);

        A[row][0] = a1;
        A[row][1] = a2;
        b[row] = GAMMA_WS * hWs;
    }

    // Simple projected gradient descent for small QP:
    // min 0.5 ||v - vNom||^2  s.t.  A v <= b
    public static double[] projectQP(double[] vNom,
                                     double[][] A,
                                     double[] b,
                                     int maxIter,
                                     double stepSize) {
        double[] v = vNom.clone();
        int m = b.length;
        for (int it = 0; it < maxIter; ++it) {
            // Gradient of objective: v - vNom
            double g0 = v[0] - vNom[0];
            double g1 = v[1] - vNom[1];
            v[0] -= stepSize * g0;
            v[1] -= stepSize * g1;

            // Project onto each half-space sequentially
            for (int i = 0; i < m; ++i) {
                double ai0 = A[i][0];
                double ai1 = A[i][1];
                double lhs = ai0 * v[0] + ai1 * v[1];
                if (lhs > b[i]) {
                    double norm2 = ai0 * ai0 + ai1 * ai1;
                    if (norm2 > 1e-8) {
                        double alpha = (lhs - b[i]) / norm2;
                        v[0] -= alpha * ai0;
                        v[1] -= alpha * ai1;
                    }
                }
            }
        }
        return v;
    }

    public static double[] safetyFilter(double[] q, double[] vNom) {
        // A has 5 rows (4 joint constraints + 1 workspace) and 2 columns
        double[][] A = new double[5][2];
        double[] b = new double[5];
        buildConstraints(q, A, b);
        return projectQP(vNom, A, b, 50, 0.1);
    }
}
