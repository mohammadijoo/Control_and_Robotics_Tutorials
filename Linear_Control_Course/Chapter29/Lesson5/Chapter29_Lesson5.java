public class PdMotorSimulation {
    public static void main(String[] args) {
        double T = 0.5;
        double zeta = 0.7;
        double tsDes = 1.0;
        double wn = 4.0 / (zeta * tsDes);

        double Kp = 0.5 * wn * wn;
        double Td = (zeta * wn - 1.0) / Kp;

        System.out.printf("Kp = %.3f, Td = %.3f, wn = %.3f%n", Kp, Td, wn);

        double h = 1e-3;
        double tEnd = 5.0;
        int N = (int) (tEnd / h);

        double x1 = 0.0;  // theta
        double x2 = 0.0;  // theta_dot
        double r = 1.0;

        for (int k = 0; k <= N; ++k) {
            double t = k * h;

            double e = r - x1;
            double u = Kp * (e - Td * x2);

            double x1Dot = x2;
            double x2Dot = (-x2 + u) / T;

            x1 += h * x1Dot;
            x2 += h * x2Dot;

            if (k % 1000 == 0) {
                System.out.printf("t = %.3f, theta = %.4f%n", t, x1);
            }
        }
    }
}
