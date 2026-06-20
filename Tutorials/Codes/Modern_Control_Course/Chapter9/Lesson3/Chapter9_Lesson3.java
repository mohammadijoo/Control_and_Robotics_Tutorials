// Chapter9_Lesson3.java
// Analytic modal decomposition for a real second-order oscillatory mode
// plus one fast real mode.
//
// Compile:
//   javac Chapter9_Lesson3.java
// Run:
//   java Chapter9_Lesson3
//
// For general eigenvalue computations in Java, libraries such as EJML,
// ojAlgo, or Apache Commons Math can be used. This file stays dependency-free
// by using a system whose modes are known analytically.

public class Chapter9_Lesson3 {
    static class State {
        double x1;
        double x2;
        double x3;

        State(double x1, double x2, double x3) {
            this.x1 = x1;
            this.x2 = x2;
            this.x3 = x3;
        }
    }

    static State fullAnalyticResponse(double sigma, double omega, double fastPole,
                                      State x0, double t) {
        double eSlow = Math.exp(sigma * t);
        double c = Math.cos(omega * t);
        double s = Math.sin(omega * t);

        // Dynamics:
        // x1_dot = sigma*x1 + omega*x2
        // x2_dot = -omega*x1 + sigma*x2
        double x1 = eSlow * (x0.x1 * c + x0.x2 * s);
        double x2 = eSlow * (-x0.x1 * s + x0.x2 * c);
        double x3 = Math.exp(fastPole * t) * x0.x3;

        return new State(x1, x2, x3);
    }

    static State dominantApproximation(double sigma, double omega, State x0, double t) {
        double eSlow = Math.exp(sigma * t);
        double c = Math.cos(omega * t);
        double s = Math.sin(omega * t);

        double x1 = eSlow * (x0.x1 * c + x0.x2 * s);
        double x2 = eSlow * (-x0.x1 * s + x0.x2 * c);

        // The fast state is neglected in the dominant-mode model.
        return new State(x1, x2, 0.0);
    }

    static double output(State x) {
        return x.x1 + 0.4 * x.x3;
    }

    public static void main(String[] args) {
        double sigma = -0.25;
        double omega = 1.50;
        double fastPole = -3.00;

        State x0 = new State(1.0, -0.2, 2.0);

        System.out.println("Eigenvalues:");
        System.out.println("  lambda_1,2 = " + sigma + " +/- " + omega + "j");
        System.out.println("  lambda_3   = " + fastPole);
        System.out.println();
        System.out.println("t, y_full, y_dominant, absolute_error");

        for (int k = 0; k <= 40; k++) {
            double t = 0.5 * k;

            State full = fullAnalyticResponse(sigma, omega, fastPole, x0, t);
            State dom = dominantApproximation(sigma, omega, x0, t);

            double yFull = output(full);
            double yDom = output(dom);
            double error = Math.abs(yFull - yDom);

            System.out.printf("%.2f, %.8f, %.8f, %.8e%n", t, yFull, yDom, error);
        }

        double settlingTimeApprox = 4.0 / Math.abs(sigma);
        double dampedPeriod = 2.0 * Math.PI / omega;

        System.out.println();
        System.out.printf("Dominant-mode settling-time approximation: %.4f s%n",
                          settlingTimeApprox);
        System.out.printf("Dominant oscillation period: %.4f s%n", dampedPeriod);
    }
}
