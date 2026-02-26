// Chapter 2 - Lesson 2: Differential Drive Kinematics (Java)
//
// Compile and run:
//   javac Chapter2_Lesson2.java
//   java Chapter2_Lesson2

public class Chapter2_Lesson2 {

    public static final class DiffDriveParams {
        public final double r; // wheel radius [m]
        public final double L; // axle length [m]
        public DiffDriveParams(double r, double L) {
            this.r = r;
            this.L = L;
        }
    }

    public static final class Pose2D {
        public final double x, y, theta;
        public Pose2D(double x, double y, double theta) {
            this.x = x; this.y = y; this.theta = theta;
        }
    }

    // wheel rates -> (v,w)
    public static double[] bodyTwistFromWheels(double phiDotL, double phiDotR, DiffDriveParams p) {
        double vL = p.r * phiDotL;
        double vR = p.r * phiDotR;
        double v  = 0.5 * (vR + vL);
        double w  = (vR - vL) / p.L;
        return new double[]{v, w};
    }

    // (v,w) -> wheel rates
    public static double[] wheelsFromBodyTwist(double v, double w, DiffDriveParams p) {
        double phiDotR = (v + 0.5 * p.L * w) / p.r;
        double phiDotL = (v - 0.5 * p.L * w) / p.r;
        return new double[]{phiDotL, phiDotR};
    }

    // exact integration for constant (v,w) over dt
    public static Pose2D integratePoseExact(Pose2D pose, double v, double w, double dt) {
        double x = pose.x, y = pose.y, th = pose.theta;

        if (Math.abs(w) < 1e-12) {
            return new Pose2D(
                    x + v * dt * Math.cos(th),
                    y + v * dt * Math.sin(th),
                    th
            );
        }

        double th2 = th + w * dt;
        double x2 = x + (v / w) * (Math.sin(th2) - Math.sin(th));
        double y2 = y - (v / w) * (Math.cos(th2) - Math.cos(th));
        return new Pose2D(x2, y2, th2);
    }

    public static Pose2D stepFromWheels(Pose2D pose, double phiDotL, double phiDotR, DiffDriveParams p, double dt) {
        double[] vw = bodyTwistFromWheels(phiDotL, phiDotR, p);
        return integratePoseExact(pose, vw[0], vw[1], dt);
    }

    public static void main(String[] args) {
        DiffDriveParams p = new DiffDriveParams(0.05, 0.30);
        Pose2D pose = new Pose2D(0.0, 0.0, 0.0);

        double phiDotL = 5.0;
        double phiDotR = 8.0;
        double dt = 0.1;
        int N = 50;

        double[] vw = bodyTwistFromWheels(phiDotL, phiDotR, p);
        System.out.printf("v = %.6f m/s, w = %.6f rad/s%n", vw[0], vw[1]);

        for (int k = 0; k < N; k++) {
            pose = stepFromWheels(pose, phiDotL, phiDotR, p, dt);
        }

        System.out.printf("Final pose [x, y, theta] = [%.6f, %.6f, %.6f]%n",
                pose.x, pose.y, pose.theta);

        double[] lr = wheelsFromBodyTwist(vw[0], vw[1], p);
        System.out.printf("Inverse check [phiDotL, phiDotR] = [%.6f, %.6f]%n",
                lr[0], lr[1]);
    }
}
