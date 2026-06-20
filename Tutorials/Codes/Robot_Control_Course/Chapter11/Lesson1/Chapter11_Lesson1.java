
public class JointObserverDemo {
    public static void main(String[] args) {
        double[][] A = { {0.0, 1.0},
                        {0.0, -0.5} };
        double[] B = {0.0, 1.0};
        double[] C = {1.0, 0.0};
        double[] K = {3.0, 2.0};
        double[] L = {5.0, 6.0};

        double Ts = 0.001;
        int steps = 5000;

        double[] x = {0.5, 0.0};
        double[] xHat = {0.0, 0.0};

        java.util.Random rng = new java.util.Random(0);

        for (int k = 0; k < steps; ++k) {
            double t = k * Ts;

            double y = C[0] * x[0] + C[1] * x[1]
                       + 0.01 * rng.nextGaussian();
            double yHat = C[0] * xHat[0] + C[1] * xHat[1];

            double u = -(K[0] * xHat[0] + K[1] * xHat[1]);

            double xdot0 = A[0][0] * x[0] + A[0][1] * x[1] + B[0] * u;
            double xdot1 = A[1][0] * x[0] + A[1][1] * x[1] + B[1] * u;

            double e = y - yHat;
            double xHatDot0 = A[0][0] * xHat[0] + A[0][1] * xHat[1]
                              + B[0] * u + L[0] * e;
            double xHatDot1 = A[1][0] * xHat[0] + A[1][1] * xHat[1]
                              + B[1] * u + L[1] * e;

            x[0] += Ts * xdot0;
            x[1] += Ts * xdot1;
            xHat[0] += Ts * xHatDot0;
            xHat[1] += Ts * xHatDot1;

            if (k % 1000 == 0) {
                System.out.printf("t=%.3f  q=%.3f  q_hat=%.3f%n",
                                  t, x[0], xHat[0]);
            }
        }
    }
}
