import java.util.ArrayList;
import java.util.List;

public class IK2R {

    public static class JointLimits {
        public double[] lower = new double[2];
        public double[] upper = new double[2];

        public JointLimits(double l1, double u1, double l2, double u2) {
            lower[0] = l1; upper[0] = u1;
            lower[1] = l2; upper[1] = u2;
        }
    }

    private static double wrapToPi(double angle) {
        double twoPi = 2.0 * Math.PI;
        angle = (angle + Math.PI) % twoPi;
        if (angle < 0.0) {
            angle += twoPi;
        }
        return angle - Math.PI;
    }

    public static List<double[]> ik2R(
            double x, double y,
            double l1, double l2,
            JointLimits limits)
    {
        List<double[]> solutions = new ArrayList<>();

        double r2 = x * x + y * y;
        double c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);

        if (Math.abs(c2) > 1.0) {
            return solutions; // empty
        }

        double s2_pos = Math.sqrt(Math.max(0.0, 1.0 - c2 * c2));
        double[] s2Candidates = new double[] { s2_pos, -s2_pos };

        for (double s2 : s2Candidates) {
            double theta2 = Math.atan2(s2, c2);
            double k1 = l1 + l2 * c2;
            double k2 = l2 * s2;
            double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1);

            double t1 = wrapToPi(theta1);
            double t2 = wrapToPi(theta2);

            if (limits != null) {
                if (t1 < limits.lower[0] || t1 > limits.upper[0] ||
                    t2 < limits.lower[1] || t2 > limits.upper[1]) {
                    continue;
                }
            }

            solutions.add(new double[] { t1, t2 });
        }

        return solutions;
    }

    public static void main(String[] args) {
        JointLimits lim = new JointLimits(-Math.PI, Math.PI, -Math.PI, Math.PI);
        List<double[]> sols = ik2R(0.5, 0.5, 0.4, 0.4, lim);
        System.out.println("Number of solutions: " + sols.size());
        for (int i = 0; i < sols.size(); ++i) {
            double[] q = sols.get(i);
            System.out.printf("Solution %d: theta1=%.3f, theta2=%.3f%n", i, q[0], q[1]);
        }
    }
}
      
