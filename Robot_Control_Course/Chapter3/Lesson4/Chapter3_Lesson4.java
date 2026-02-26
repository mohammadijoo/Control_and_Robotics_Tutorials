
import org.ejml.simple.SimpleMatrix;

public class JointFilter {
    private final double Ts;
    private final double alphaVel;
    private final double betaPos;
    private SimpleMatrix qPrev;
    private SimpleMatrix qf;
    private SimpleMatrix qdotEst;
    private final boolean filterPos;

    public JointFilter(int n, double Ts, double omegaCVel, Double omegaCPos) {
        this.Ts = Ts;
        this.alphaVel = 1.0 / (1.0 + omegaCVel * Ts);
        this.qPrev = new SimpleMatrix(n, 1);
        this.qf = new SimpleMatrix(n, 1);
        this.qdotEst = new SimpleMatrix(n, 1);
        if (omegaCPos != null && omegaCPos > 0.0) {
            this.betaPos = 1.0 / (1.0 + omegaCPos * Ts);
            this.filterPos = true;
        } else {
            this.betaPos = -1.0;
            this.filterPos = false;
        }
    }

    public void update(SimpleMatrix qMeas) {
        if (filterPos) {
            qf = qf.scale(betaPos).plus(qMeas.scale(1.0 - betaPos));
        } else {
            qf = qMeas.copy();
        }
        SimpleMatrix dqRaw = qMeas.minus(qPrev).divide(Ts);
        qdotEst = qdotEst.scale(alphaVel)
                .plus(dqRaw.scale(1.0 - alphaVel));
        qPrev = qMeas.copy();
    }

    public SimpleMatrix getQf() { return qf; }
    public SimpleMatrix getQdotEst() { return qdotEst; }
}

public class Saturation {
    public static SimpleMatrix saturate(SimpleMatrix u,
                                        SimpleMatrix uMin,
                                        SimpleMatrix uMax) {
        SimpleMatrix out = u.copy();
        for (int i = 0; i < u.getNumElements(); ++i) {
            double val = out.get(i);
            val = Math.min(Math.max(val, uMin.get(i)), uMax.get(i));
            out.set(i, val);
        }
        return out;
    }

    public static SimpleMatrix rateLimit(SimpleMatrix uCmd,
                                         SimpleMatrix uPrev,
                                         SimpleMatrix duMin,
                                         SimpleMatrix duMax) {
        SimpleMatrix out = uPrev.copy();
        for (int i = 0; i < uCmd.getNumElements(); ++i) {
            double du = uCmd.get(i) - uPrev.get(i);
            du = Math.min(Math.max(du, duMin.get(i)), duMax.get(i));
            out.set(i, uPrev.get(i) + du);
        }
        return out;
    }
}

public interface RobotModel {
    SimpleMatrix M(SimpleMatrix q);
    SimpleMatrix C(SimpleMatrix q, SimpleMatrix qdot);
    SimpleMatrix g(SimpleMatrix q);
}

public class ComputedTorqueController {
    private final RobotModel robot;
    private final SimpleMatrix Kp;
    private final SimpleMatrix Kd;
    private final double Ts;
    private final SimpleMatrix tauMin;
    private final SimpleMatrix tauMax;
    private final SimpleMatrix dtauMin;
    private final SimpleMatrix dtauMax;
    private final JointFilter filter;
    private SimpleMatrix tauPrev;

    public ComputedTorqueController(RobotModel robot,
                                    SimpleMatrix KpDiag,
                                    SimpleMatrix KdDiag,
                                    double Ts,
                                    SimpleMatrix tauMin,
                                    SimpleMatrix tauMax,
                                    SimpleMatrix dtauMin,
                                    SimpleMatrix dtauMax,
                                    double omegaCVel,
                                    Double omegaCPos) {
        this.robot = robot;
        this.Kp = SimpleMatrix.diag(KpDiag.getDDRM());
        this.Kd = SimpleMatrix.diag(KdDiag.getDDRM());
        this.Ts = Ts;
        this.tauMin = tauMin;
        this.tauMax = tauMax;
        this.dtauMin = dtauMin;
        this.dtauMax = dtauMax;
        int n = tauMin.getNumElements();
        this.filter = new JointFilter(n, Ts, omegaCVel, omegaCPos);
        this.tauPrev = new SimpleMatrix(n, 1);
    }

    public SimpleMatrix step(SimpleMatrix qMeas,
                             SimpleMatrix qd,
                             SimpleMatrix qdDot,
                             SimpleMatrix qdDdot) {
        filter.update(qMeas);
        SimpleMatrix qf = filter.getQf();
        SimpleMatrix qdotEst = filter.getQdotEst();

        SimpleMatrix eQ = qd.minus(qf);
        SimpleMatrix eQdot = qdDot.minus(qdotEst);

        SimpleMatrix M = robot.M(qf);
        SimpleMatrix C = robot.C(qf, qdotEst);
        SimpleMatrix g = robot.g(qf);

        SimpleMatrix v = qdDdot
                .plus(Kd.mult(eQdot))
                .plus(Kp.mult(eQ));
        SimpleMatrix tauPre = M.mult(v).plus(C.mult(qdotEst)).plus(g);

        SimpleMatrix tauRl = Saturation.rateLimit(tauPre, tauPrev, dtauMin, dtauMax);
        SimpleMatrix tauSat = Saturation.saturate(tauRl, tauMin, tauMax);
        tauPrev = tauSat.copy();
        return tauSat;
    }
}
