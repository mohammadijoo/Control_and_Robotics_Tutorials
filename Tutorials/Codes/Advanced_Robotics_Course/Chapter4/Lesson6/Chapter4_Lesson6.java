public class TwoLinkCollocationProblem {

    private final int N;
    private final double T;
    private final double h;
    private final int nx = 4;
    private final int nu = 2;

    public TwoLinkCollocationProblem(int N, double T) {
        this.N = N;
        this.T = T;
        this.h = T / N;
    }

    // Map decision vector z to (x_k, u_k)
    private int indexX(int k, int i) {
        // x_k: i = 0..3
        return k * (nx + nu) + i;
    }

    private int indexU(int k, int i) {
        // u_k: i = 0..1
        return k * (nx + nu) + nx + i;
    }

    public double objective(double[] z) {
        double cost = 0.0;
        for (int k = 0; k <= N; ++k) {
            double u1 = z[indexU(k, 0)];
            double u2 = z[indexU(k, 1)];
            cost += 0.5 * h * (u1 * u1 + u2 * u2);
        }
        return cost;
    }

    // Equality constraints: boundary + collocation
    public double[] constraints(double[] z) {
        int m = (N + 1) * nx; // upper bound; some rows unused if needed
        double[] g = new double[m];
        int row = 0;

        double[] xInit = {0.0, 0.0, 0.0, 0.0};
        double[] xGoal = {Math.PI / 2.0, 0.0, 0.0, 0.0};

        // x_0 - xInit = 0
        for (int i = 0; i < nx; ++i) {
            g[row++] = z[indexX(0, i)] - xInit[i];
        }

        // x_N - xGoal = 0
        for (int i = 0; i < nx; ++i) {
            g[row++] = z[indexX(N, i)] - xGoal[i];
        }

        // Collocation constraints
        for (int k = 0; k < N; ++k) {
            double[] xk = new double[nx];
            double[] xk1 = new double[nx];
            double[] uk = new double[nu];
            double[] uk1 = new double[nu];

            for (int i = 0; i < nx; ++i) {
                xk[i] = z[indexX(k, i)];
                xk1[i] = z[indexX(k + 1, i)];
            }
            for (int i = 0; i < nu; ++i) {
                uk[i] = z[indexU(k, i)];
                uk1[i] = z[indexU(k + 1, i)];
            }

            double[] fk = twoLinkDynamics(xk, uk);
            double[] fk1 = twoLinkDynamics(xk1, uk1);

            for (int i = 0; i < nx; ++i) {
                double defect = xk1[i] - xk[i] - 0.5 * h * (fk[i] + fk1[i]);
                g[row++] = defect;
            }
        }

        // If the library expects g.length == number of constraints,
        // one can truncate or size m accordingly.
        return java.util.Arrays.copyOf(g, row);
    }

    private double[] twoLinkDynamics(double[] x, double[] u) {
        // Same dynamics as in Python example, implemented in Java.
        double q1 = x[0], q2 = x[1];
        double v1 = x[2], v2 = x[3];
        double u1 = u[0], u2 = u[1];

        double m1 = 1.0, m2 = 1.0;
        double l1 = 1.0, l2 = 1.0;
        double lc1 = 0.5, lc2 = 0.5;
        double I1 = 0.12, I2 = 0.12;
        double g = 9.81;

        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);

        double M11 = I1 + I2 + m2 * l1 * l1 + 2.0 * m2 * l1 * lc2 * c2;
        double M12 = I2 + m2 * l1 * lc2 * c2;
        double M21 = M12;
        double M22 = I2;

        double hC = m2 * l1 * lc2 * s2;
        double C11 = 0.0;
        double C12 = -2.0 * hC * v2;
        double C21 = hC * v1;
        double C22 = 0.0;

        double g1 = (m1 * lc1 + m2 * l1) * g * Math.cos(q1)
                    + m2 * lc2 * g * Math.cos(q1 + q2);
        double g2 = m2 * lc2 * g * Math.cos(q1 + q2);

        double v1dot, v2dot;
        // Solve 2x2 linear system M * a = tau - C * v - g
        double detM = M11 * M22 - M12 * M21;
        double v1c = C11 * v1 + C12 * v2;
        double v2c = C21 * v1 + C22 * v2;

        double rhs1 = u1 - v1c - g1;
        double rhs2 = u2 - v2c - g2;

        v1dot = ( M22 * rhs1 - M12 * rhs2 ) / detM;
        v2dot = (-M21 * rhs1 + M11 * rhs2 ) / detM;

        return new double[] {v1, v2, v1dot, v2dot};
    }
}
      
