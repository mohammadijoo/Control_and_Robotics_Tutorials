
public interface Controller {
    double[] computeTau(double t,
                        double[] q,
                        double[] dq,
                        Reference ref);
}

public class Reference {
    public double[] qd;
    public double[] dqd;
    public double[] ddqd;
}

public class JointPD implements Controller {
    private final double[] kp;
    private final double[] kd;

    public JointPD(double[] kp, double[] kd) {
        this.kp = kp;
        this.kd = kd;
    }

    @Override
    public double[] computeTau(double t,
                               double[] q,
                               double[] dq,
                               Reference ref) {
        int n = q.length;
        double[] tau = new double[n];
        for (int i = 0; i < n; ++i) {
            double e  = ref.qd[i]  - q[i];
            double de = ref.dqd[i] - dq[i];
            tau[i] = kp[i] * e + kd[i] * de;
        }
        return tau;
    }
}

public class SafetyFilter {
    private final double[] umin;
    private final double[] umax;

    public SafetyFilter(double[] umin, double[] umax) {
        this.umin = umin;
        this.umax = umax;
    }

    public double[] filter(double[] uNom) {
        int n = uNom.length;
        double[] u = new double[n];
        for (int i = 0; i < n; ++i) {
            double v = uNom[i];
            if (v < umin[i]) v = umin[i];
            if (v > umax[i]) v = umax[i];
            u[i] = v;
        }
        return u;
    }
}

public class Architecture {
    private final Controller controller;
    private final SafetyFilter safety;

    public Architecture(Controller controller, SafetyFilter safety) {
        this.controller = controller;
        this.safety = safety;
    }

    public double[] computeTau(double t,
                               double[] q,
                               double[] dq,
                               Reference ref) {
        double[] uNom = controller.computeTau(t, q, dq, ref);
        if (safety != null) {
            return safety.filter(uNom);
        }
        return uNom;
    }
}
