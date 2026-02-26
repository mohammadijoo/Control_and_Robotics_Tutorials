import java.util.ArrayList;
import java.util.List;

public class IK2R {
    public static class AnglePair {
        public double theta1;
        public double theta2;
        public AnglePair(double t1, double t2) {
            this.theta1 = t1;
            this.theta2 = t2;
        }
    }

    public static List<AnglePair> solve(double x, double y, double l1, double l2, double tol) {
        List<AnglePair> sols = new ArrayList<>();
        double r2 = x*x + y*y;
        if (r2 > (l1 + l2)*(l1 + l2) + tol || r2 < (l1 - l2)*(l1 - l2) - tol) {
            return sols;
        }
        double c2 = (r2 - l1*l1 - l2*l2) / (2.0 * l1 * l2);
        c2 = Math.max(-1.0, Math.min(1.0, c2));
        double s2Abs = Math.sqrt(Math.max(0.0, 1.0 - c2*c2));
        double phi = Math.atan2(y, x);

        double[] s2Options = new double[]{ s2Abs, -s2Abs };
        for (double s2 : s2Options) {
            double theta2 = Math.atan2(s2, c2);
            double k1 = l1 + l2 * c2;
            double k2 = l2 * s2;
            double psi = Math.atan2(k2, k1);
            double theta1 = phi - psi;
            sols.add(new AnglePair(theta1, theta2));
        }
        return sols;
    }
}
      
