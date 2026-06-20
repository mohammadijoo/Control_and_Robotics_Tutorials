
public class CBF1D {

    public static double nominalControl(double x, double xRef, double kP) {
        return -kP * (x - xRef);
    }

    public static double cbfFilter1D(double x, double uNom, double gamma) {
        // CBF constraint: u >= -gamma * x
        double uMin = -gamma * x;
        return Math.max(uNom, uMin);
    }

    public static void main(String[] args) {
        double dt = 0.001;
        double T = 2.0;
        int N = (int) (T / dt);

        double x = 0.05;
        double xRef = -1.0;
        double kP = 2.0;
        double gamma = 5.0;

        for (int k = 0; k < N; ++k) {
            double uNom = nominalControl(x, xRef, kP);
            double uSafe = cbfFilter1D(x, uNom, gamma);

            x += dt * uSafe;

            if (k % 100 == 0) {
                System.out.println("t = " + (k * dt)
                        + ", x = " + x
                        + ", u_nom = " + uNom
                        + ", u_safe = " + uSafe);
            }
        }
    }
}
