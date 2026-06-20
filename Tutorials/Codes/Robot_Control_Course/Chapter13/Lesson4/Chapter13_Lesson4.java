
public class SafeIntegratorController {

    private final double uMin;
    private final double uMax;
    private final double kappa;
    private final double kP;

    public SafeIntegratorController(double uMin, double uMax, double kappa, double kP) {
        this.uMin = uMin;
        this.uMax = uMax;
        this.kappa = kappa;
        this.kP = kP;
    }

    // Safety filter for dot{x} = u with h(x) = x.
    public double safeInput(double x, double uDes) {
        double uLow = Math.max(uMin, -kappa * x);
        double uHigh = uMax;
        double uSafe = Math.min(uHigh, Math.max(uLow, uDes));
        return uSafe;
    }

    public void simulate(double x0, double T, double dt) {
        double x = x0;
        int N = (int) (T / dt);
        double xRef = 1.0;

        for (int k = 0; k < N; ++k) {
            double t = (k + 1) * dt;
            double uDes = -kP * (x - xRef);
            double uSafe = safeInput(x, uDes);
            x = x + dt * uSafe;

            if (k % 100 == 0) {
                System.out.println("t = " + t
                        + ", x = " + x
                        + ", u_des = " + uDes
                        + ", u_safe = " + uSafe);
            }
        }
    }

    public static void main(String[] args) {
        SafeIntegratorController ctrl =
                new SafeIntegratorController(-2.0, 2.0, 5.0, 5.0);
        ctrl.simulate(0.1, 2.0, 0.001);
    }
}
