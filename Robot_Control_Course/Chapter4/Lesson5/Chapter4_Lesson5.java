
public class TaskSpaceController2DOF {

    private double dt;
    private double[][] Kp;
    private double[][] Kd;

    public TaskSpaceController2DOF(double dt) {
        this.dt = dt;
        this.Kp = new double[][]{  {100.0, 0.0}, {0.0, 100.0}  };
        this.Kd = new double[][]{  {20.0, 0.0}, {0.0, 20.0}  };
    }

    // Implement kinematics, Jacobian, etc. similarly to the Python code
    public double[] forwardKinematics(double[] q) { /* ... */ return new double[2]; }
    public double[][] jacobian(double[] q) { /* ... */ return new double[2][2]; }
    public double[][] jacobianDot(double[] q, double[] qdot) { /* ... */ return new double[2][2]; }

    public double[] xDes(double t) { /* quintic line */ return new double[2]; }
    public double[] xdotDes(double t) { return new double[2]; }
    public double[] xddotDes(double t) { return new double[2]; }

    public double[] step(double t, double[] q, double[] qdot) {
        double[] x = forwardKinematics(q);
        double[][] J = jacobian(q);
        double[] xdot = matVec(J, qdot);
        double[][] Jdot = jacobianDot(q, qdot);

        double[] xd = xDes(t);
        double[] xdDot = xdotDes(t);
        double[] xdd = xddotDes(t);

        double[] e = new double[]{xd[0] - x[0], xd[1] - x[1]};
        double[] eDot = new double[]{xdDot[0] - xdot[0], xdDot[1] - xdot[1]};

        double[] Kp_e = matVec(Kp, e);
        double[] Kd_eDot = matVec(Kd, eDot);

        double[] a_x = new double[]{
            xdd[0] + Kd_eDot[0] + Kp_e[0],
            xdd[1] + Kd_eDot[1] + Kp_e[1]
        };

        double[][] Jinv = invert2x2(J);
        double[] Jdot_qdot = matVec(Jdot, qdot);
        double[] rhs = new double[]{
            a_x[0] - Jdot_qdot[0],
            a_x[1] - Jdot_qdot[1]
        };

        double[] qdd_des = matVec(Jinv, rhs);
        return qdd_des; // to be consumed by lower-level torque or velocity controller
    }

    private double[] matVec(double[][] A, double[] x) { /* ... */ return new double[x.length]; }
    private double[][] invert2x2(double[][] A) { /* closed-form inverse */ return new double[2][2]; }
}
