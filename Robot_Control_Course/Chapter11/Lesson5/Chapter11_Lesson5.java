
public class ObserverBasedControlLab {

    public static void main(String[] args) {

        double M = 2.0;
        double b = 0.8;
        double k = 5.0;

        // A matrix entries
        double a11 = 0.0;
        double a12 = 1.0;
        double a21 = -k / M;
        double a22 = -b / M;

        // B vector
        double b1 = 0.0;
        double b2 = 1.0 / M;

        // C row vector
        double c1 = 1.0;
        double c2 = 0.0;

        // State-feedback gains K = [k1 k2]
        double k1 = 27.0;
        double k2 = 13.6;

        // Observer gains L = [l1; l2]
        double l1 = 28.4;
        double l2 = 242.14;

        double dt = 1e-3;
        double T_end = 5.0;
        int N = (int) (T_end / dt);

        double qRef = 0.5;
        double xRef1 = qRef;
        double xRef2 = 0.0;

        double x1 = 0.0;     // q
        double x2 = 0.0;     // q_dot
        double xhat1 = 0.0;  // q_hat
        double xhat2 = 0.0;  // q_dot_hat

        for (int kStep = 0; kStep != N; ++kStep) {

            double y = c1 * x1 + c2 * x2;
            double yHat = c1 * xhat1 + c2 * xhat2;
            double yErr = y - yHat;

            double u = -(k1 * (xhat1 - xRef1) + k2 * (xhat2 - xRef2));

            double xdot1 = a11 * x1 + a12 * x2 + b1 * u;
            double xdot2 = a21 * x1 + a22 * x2 + b2 * u;

            double xhatdot1 = a11 * xhat1 + a12 * xhat2 + b1 * u + l1 * yErr;
            double xhatdot2 = a21 * xhat1 + a22 * xhat2 + b2 * u + l2 * yErr;

            x1 += dt * xdot1;
            x2 += dt * xdot2;
            xhat1 += dt * xhatdot1;
            xhat2 += dt * xhatdot2;
        }

        System.out.println("Final q     = " + x1);
        System.out.println("Final q_hat = " + xhat1);
    }
}
