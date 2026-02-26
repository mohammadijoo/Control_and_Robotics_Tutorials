
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

interface RobotModelJava {
    DMatrixRMaj forwardKinematics(DMatrixRMaj q);      // 3x1
    DMatrixRMaj jacobian(DMatrixRMaj q);               // 3xn
    DMatrixRMaj inverseDynamics(DMatrixRMaj q,
                                DMatrixRMaj qdot,
                                DMatrixRMaj qddot);    // nx1
    DMatrixRMaj forceSensor();                         // 3x1
}

public class HybridPositionForceControllerJava {

    private RobotModelJava robot;
    private DMatrixRMaj n;     // 3x1
    private DMatrixRMaj S_p;   // 3x3
    private DMatrixRMaj S_f;   // 3x3
    private DMatrixRMaj Kp_tan;
    private DMatrixRMaj Kd_tan;
    private double kf;

    public HybridPositionForceControllerJava(RobotModelJava robot,
                                             DMatrixRMaj contactNormalWorld) {
        this.robot = robot;
        this.n = contactNormalWorld.copy();
        normalize(n);

        S_f = new DMatrixRMaj(3, 3);
        outer(n, n, S_f);
        S_p = CommonOps_DDRM.identity(3);
        CommonOps_DDRM.subtractEquals(S_p, S_f);

        Kp_tan = CommonOps_DDRM.identity(3);
        CommonOps_DDRM.scale(200.0, Kp_tan);
        Kd_tan = CommonOps_DDRM.identity(3);
        CommonOps_DDRM.scale(40.0, Kd_tan);
        kf = 50.0;
    }

    public DMatrixRMaj computeTorque(DMatrixRMaj q,
                                     DMatrixRMaj qdot,
                                     DMatrixRMaj x_d,
                                     DMatrixRMaj xdot_d,
                                     double Fd_n) {

        DMatrixRMaj x = robot.forwardKinematics(q);
        DMatrixRMaj J = robot.jacobian(q);          // 3xn
        DMatrixRMaj xdot = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.mult(J, qdot, xdot);

        // Project desired position to tangent plane
        DMatrixRMaj delta = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.subtract(x_d, x, delta);
        double nTdelta = dot(n, delta);
        DMatrixRMaj normalComponent = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.scale(nTdelta, n, normalComponent);
        DMatrixRMaj x_d_proj = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.subtract(x_d, normalComponent, x_d_proj);

        DMatrixRMaj e_p = new DMatrixRMaj(3, 1);
        DMatrixRMaj tmp = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.subtract(x_d_proj, x, tmp);
        CommonOps_DDRM.mult(S_p, tmp, e_p);

        DMatrixRMaj e_dot_p = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.subtract(xdot_d, xdot, tmp);
        CommonOps_DDRM.mult(S_p, tmp, e_dot_p);

        DMatrixRMaj F_t_cmd = new DMatrixRMaj(3, 1);
        DMatrixRMaj tmp3 = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.mult(Kp_tan, e_p, F_t_cmd);
        CommonOps_DDRM.multAdd(Kd_tan, e_dot_p, F_t_cmd);
        CommonOps_DDRM.mult(S_p, F_t_cmd, tmp3);
        F_t_cmd.set(tmp3);

        // Force loop
        DMatrixRMaj F_meas = robot.forceSensor();
        double F_n = dot(n, F_meas);
        double e_f = Fd_n - F_n;
        double F_n_cmd = Fd_n + kf * e_f;

        DMatrixRMaj F_n_vec = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.scale(F_n_cmd, n, F_n_vec);

        DMatrixRMaj F_cmd = new DMatrixRMaj(3, 1);
        CommonOps_DDRM.add(F_t_cmd, F_n_vec, F_cmd);

        // Map to joint torques tau = J^T F_cmd + bias
        DMatrixRMaj tau = new DMatrixRMaj(q.getNumRows(), 1);
        CommonOps_DDRM.multTransA(J, F_cmd, tau);

        DMatrixRMaj qddotZero = new DMatrixRMaj(q.getNumRows(), 1);
        DMatrixRMaj tauBias = robot.inverseDynamics(q, qdot, qddotZero);
        CommonOps_DDRM.addEquals(tau, tauBias);

        return tau;
    }

    // Helper methods
    private static void normalize(DMatrixRMaj v) {
        double norm = Math.sqrt(dot(v, v));
        CommonOps_DDRM.scale(1.0 / norm, v);
    }

    private static double dot(DMatrixRMaj a, DMatrixRMaj b) {
        double s = 0.0;
        for (int i = 0; i < a.getNumRows(); ++i) {
            s += a.get(i, 0) * b.get(i, 0);
        }
        return s;
    }

    private static void outer(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj out) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                out.set(i, j, a.get(i, 0) * b.get(j, 0));
            }
        }
    }
}
