public class IK2R {

    public static class Solution {
        public final double q1;
        public final double q2;
        public Solution(double q1, double q2) {
            this.q1 = q1;
            this.q2 = q2;
        }
    }

    public static java.util.List<Solution> solve(double l1, double l2,
                                                   double x_d, double y_d,
                                                   double tol) {
        java.util.List<Solution> sols = new java.util.ArrayList<>();

        double d_sq = x_d * x_d + y_d * y_d;
        double d = Math.sqrt(d_sq);

        if (d > l1 + l2 + tol || d < Math.abs(l1 - l2) - tol) {
            return sols; // unreachable
        }

        double c2 = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
        c2 = Math.max(-1.0, Math.min(1.0, c2));

        double disc = Math.max(0.0, 1.0 - c2 * c2);
        double[] s2Candidates = { Math.sqrt(disc), -Math.sqrt(disc) };

        double phi = Math.atan2(y_d, x_d);

        for (double s2 : s2Candidates) {
            double q2 = Math.atan2(s2, c2);
            double k1 = l1 + l2 * c2;
            double k2 = l2 * s2;
            double psi = Math.atan2(k2, k1);

            double q1 = phi - psi;
            sols.add(new Solution(q1, q2));
        }
        return sols;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.6;
        double x_d = 1.0, y_d = 0.4;

        java.util.List<Solution> sols = solve(l1, l2, x_d, y_d, 1e-9);
        if (sols.isEmpty()) {
            System.out.println("Target unreachable");
        } else {
            for (int i = 0; i < sols.size(); ++i) {
                Solution s = sols.get(i);
                System.out.printf("Solution %d: q1 = %.3f, q2 = %.3f%n",
                                  i, s.q1, s.q2);
            }
        }
    }
}
      
