
public class ContactSafeLab {

    public static void main(String[] args) {
        double m = 1.0;
        double kEnv = 5000.0;
        double fSafe = 80.0;
        double aBrake = 30.0;
        double vMax = 1.0;

        double vForce = fSafe / Math.sqrt(m * kEnv);

        double h = 0.0005;
        double T = 1.0;
        int N = (int) (T / h);

        double xTarget = 0.0;
        double kP = 50.0;
        double kD = 5.0;
        double kV = 200.0;

        double x = 0.25;
        double v = 0.0;

        double maxPenetration = x;
        double maxForce = 0.0;

        for (int k = 0; k < N; k++) {
            double d = x;
            double vDes = -kP * (x - xTarget) - kD * v;

            double vBrake = Math.sqrt(Math.max(0.0, 2.0 * aBrake * d));
            double vSafeBound = Math.min(vForce, Math.min(vBrake, vMax));

            double vSafeCmd = vDes;
            if (vDes < -vSafeBound) {
                vSafeCmd = -vSafeBound;
            }

            double u = m * kV * (vSafeCmd - v);

            v = v + (h / m) * u;
            x = x + h * v;

            double F = 0.0;
            if (x < 0.0) {
                F = -kEnv * x;
            }

            if (x < maxPenetration) {
                maxPenetration = x;
            }
            if (Math.abs(F) > maxForce) {
                maxForce = Math.abs(F);
            }
        }

        System.out.println("Max penetration (m): " + maxPenetration);
        System.out.println("Max contact force (N): " + maxForce);
        System.out.println("Theoretical bound F_safe (N): " + fSafe);
    }
}
