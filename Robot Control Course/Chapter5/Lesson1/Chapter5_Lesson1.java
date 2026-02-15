
public class PlanarArm {
    double l1 = 1.0, l2 = 0.7;
    double q1Min = -Math.PI/2.0, q1Max = Math.PI/2.0;
    double q2Min = -Math.PI,      q2Max = Math.PI;
    double xObs = 0.7, yObs = 0.3, rObs = 0.2;

    public double[] forwardKinematics(double[] q) {
        double q1 = q[0], q2 = q[1];
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        return new double[]{x, y};
    }

    public boolean inJointLimits(double[] q) {
        double q1 = q[0], q2 = q[1];
        return (q1Min <= q1 && q1 <= q1Max) &&
               (q2Min <= q2 && q2 <= q2Max);
    }

    public double obstacleConstraint(double[] q) {
        double[] ee = forwardKinematics(q);
        double dx = ee[0] - xObs;
        double dy = ee[1] - yObs;
        return dx*dx + dy*dy - rObs*rObs; // >= 0 in free space
    }

    public double groundConstraint(double[] q) {
        double[] ee = forwardKinematics(q);
        return ee[1]; // y >= 0
    }

    public boolean admissible(double[] q) {
        if (!inJointLimits(q)) return false;
        if (obstacleConstraint(q) < 0.0) return false;
        if (groundConstraint(q) < 0.0) return false;
        return true;
    }

    public static void main(String[] args) {
        PlanarArm arm = new PlanarArm();
        double[] q = {0.0, 0.0};
        System.out.println("Admissible? " + arm.admissible(q));
    }
}
