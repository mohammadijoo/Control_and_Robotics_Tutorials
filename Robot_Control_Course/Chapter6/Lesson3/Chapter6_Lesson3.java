
public class JointImpedanceController {
    private double J;  // inertia
    private double K;  // stiffness
    private double D;  // damping

    public JointImpedanceController(double J, double K, double D) {
        this.J = J;
        this.K = K;
        this.D = D;
    }

    public double computeTorque(double q, double qd,
                                double qRef, double qdRef, double qddRef,
                                double tauExt) {
        double e = q - qRef;
        double ed = qd - qdRef;
        double tauM = J * qddRef - K * e - D * ed;
        return tauM;
    }

    public static void main(String[] args) {
        JointImpedanceController ctrl =
            new JointImpedanceController(0.05, 10.0, 2.0);

        double q = 0.0;
        double qd = 0.0;
        double dt = 0.001;
        double T = 1.0;
        int N = (int) (T / dt);

        for (int k = 0; k < N; ++k) {
            double t = k * dt;
            double qRef = (t >= 0.1) ? 0.5 : 0.0;
            double qdRef = 0.0;
            double qddRef = 0.0;

            double tauExt = (t >= 0.3 && t <= 0.35) ? 0.2 : 0.0;

            double tauM = ctrl.computeTorque(q, qd, qRef, qdRef, qddRef, tauExt);

            double qdd = (tauM + tauExt) / ctrl.J;

            qd += dt * qdd;
            q += dt * qd;
        }

        System.out.println("Final q = " + q);
    }
}
