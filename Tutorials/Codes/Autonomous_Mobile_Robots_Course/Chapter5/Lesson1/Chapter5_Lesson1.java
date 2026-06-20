// Chapter5_Lesson1.java
// Wheel Odometry Computation — Differential Drive (planar SE(2))
//
// Compile & run:
//   javac Chapter5_Lesson1.java
//   java Chapter5_Lesson1
//
// Notes:
// - For production robotics middleware, consider ROS 2 Java (rcljava) for message publishing.
// - This file stays middleware-agnostic and focuses on core odometry math.

public class Chapter5_Lesson1 {

    public static class Pose2D {
        public double x, y, theta; // yaw [rad]
        public Pose2D(double x, double y, double theta) { this.x = x; this.y = y; this.theta = theta; }
        @Override public String toString() {
            return String.format("Pose2D(x=%.6f, y=%.6f, theta=%.6f rad)", x, y, theta);
        }
    }

    public static class DiffDriveParams {
        public double rL;          // left wheel radius [m]
        public double rR;          // right wheel radius [m]
        public double b;           // baseline [m]
        public int ticksPerRev;    // encoder ticks per motor rev
        public double gearRatio;   // motor rev per wheel rev
        public DiffDriveParams(double rL, double rR, double b, int ticksPerRev, double gearRatio) {
            this.rL = rL; this.rR = rR; this.b = b; this.ticksPerRev = ticksPerRev; this.gearRatio = gearRatio;
        }
    }

    public static double normalizeAngle(double theta) {
        return Math.atan2(Math.sin(theta), Math.cos(theta));
    }

    public static class DifferentialDriveOdometry {
        private final DiffDriveParams p;
        private Pose2D pose;

        public DifferentialDriveOdometry(DiffDriveParams p, Pose2D pose0) {
            this.p = p;
            this.pose = pose0;
        }

        public double ticksToWheelAngle(int dN) {
            double denom = ((double)p.ticksPerRev) * p.gearRatio;
            return 2.0 * Math.PI * (((double)dN) / denom);
        }

        public Pose2D updateFromTicks(int dN_L, int dN_R) {
            double dphiL = ticksToWheelAngle(dN_L);
            double dphiR = ticksToWheelAngle(dN_R);

            double dsL = p.rL * dphiL;
            double dsR = p.rR * dphiR;

            double deltaS = 0.5 * (dsR + dsL);
            double deltaTheta = (dsR - dsL) / p.b;

            double th = pose.theta;
            double eps = 1e-12;

            if (Math.abs(deltaTheta) < eps) {
                pose.x += deltaS * Math.cos(th);
                pose.y += deltaS * Math.sin(th);
                pose.theta = normalizeAngle(th + deltaTheta);
            } else {
                double R = deltaS / deltaTheta;
                pose.x += R * (Math.sin(th + deltaTheta) - Math.sin(th));
                pose.y += -R * (Math.cos(th + deltaTheta) - Math.cos(th));
                pose.theta = normalizeAngle(th + deltaTheta);
            }
            return pose;
        }

        public Pose2D pose() { return pose; }
    }

    public static void main(String[] args) {
        DiffDriveParams p = new DiffDriveParams(0.05, 0.05, 0.30, 2048, 1.0);
        DifferentialDriveOdometry odo = new DifferentialDriveOdometry(p, new Pose2D(0.0, 0.0, 0.0));

        for (int k = 0; k < 200; k++) {
            odo.updateFromTicks(40, 60);
        }

        System.out.println("Final pose: " + odo.pose());
    }
}
