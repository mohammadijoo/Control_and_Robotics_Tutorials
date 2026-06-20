
public class SafeSet2D {

    public static double[] forwardKinematics2D(double[] q, double l1, double l2) {
        double q1 = q[0];
        double q2 = q[1];
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        return new double[]{x, y};
    }

    public static double[] jointLimitBarrier(double[] q,
                                             double[] qMin,
                                             double[] qMax,
                                             double margin) {
        double[] h = new double[4];
        // low limits
        h[0] = q[0] - (qMin[0] + margin);
        h[1] = q[1] - (qMin[1] + margin);
        // upper limits
        h[2] = (qMax[0] - margin) - q[0];
        h[3] = (qMax[1] - margin) - q[1];
        return h;
    }

    public static double obstacleBarrier(double[] q,
                                         double l1, double l2,
                                         double[] c,
                                         double rObs, double dMin) {
        double[] p = forwardKinematics2D(q, l1, l2);
        double dx = p[0] - c[0];
        double dy = p[1] - c[1];
        double dist2 = dx * dx + dy * dy;
        double rho = rObs + dMin;
        return dist2 - rho * rho;
    }

    public static boolean isSafe(double[] q,
                                 double[] qMin,
                                 double[] qMax,
                                 double l1, double l2,
                                 double[] c,
                                 double rObs, double dMin,
                                 double margin,
                                 double tol) {
        double[] hJoints = jointLimitBarrier(q, qMin, qMax, margin);
        double hObs = obstacleBarrier(q, l1, l2, c, rObs, dMin);
        for (int i = 0; i < hJoints.length; ++i) {
            if (hJoints[i] < -tol) {
                return false;
            }
        }
        if (hObs < -tol) {
            return false;
        }
        return true;
    }

    public static void main(String[] args) {
        double[] q = {0.0, 0.0};
        double[] qMin = {-1.0, -1.0};
        double[] qMax = { 1.0,  1.0};
        double l1 = 0.8;
        double l2 = 0.6;
        double[] c = {0.8, 0.0};
        double rObs = 0.1;
        double dMin = 0.05;
        double margin = 0.0;
        double tol = 0.0;

        System.out.println("Is safe? " +
            isSafe(q, qMin, qMax, l1, l2, c, rObs, dMin, margin, tol));
    }
}
