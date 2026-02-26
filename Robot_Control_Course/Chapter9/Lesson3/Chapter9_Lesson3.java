
public class ILQRPendulum {

    static final double m = 1.0, l = 1.0, g = 9.81, dt = 0.01;

    static double[] dynamics(double[] x, double u) {
        double theta = x[0];
        double dtheta = x[1];
        double ddtheta = (-g / l) * Math.sin(theta) + u / (m * l * l);
        double[] xnext = new double[2];
        xnext[0] = theta + dt * dtheta;
        xnext[1] = dtheta + dt * ddtheta;
        return xnext;
    }

    static void ilqr(double[] x0, double[][] xRef, int N, int maxIter) {
        double[][] X = new double[N + 1][2];
        double[] U = new double[N];
        X[0] = x0.clone();

        // Initial rollout
        for (int k = 0; k < N; ++k) {
            X[k + 1] = dynamics(X[k], U[k]);
        }

        for (int it = 0; it < maxIter; ++it) {
            // Backward and forward passes would go here,
            // using EJML matrices for Q-derivatives and gains K, k.
            // This skeleton omits details to focus on the structure.
        }
    }

    public static void main(String[] args) {
        int N = 300;
        double[] x0 = {0.5, 0.0};
        double[][] xRef = new double[N + 1][2]; // all zeros
        ilqr(x0, xRef, N, 50);
    }
}
