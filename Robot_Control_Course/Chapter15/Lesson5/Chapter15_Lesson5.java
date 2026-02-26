
import org.ejml.simple.SimpleMatrix;

public class ResidualController1DOF {
    private final double mNom;
    private final double bNom;
    private final double gNom;
    private final double Kp;
    private final double Kd;
    private final SimpleMatrix w; // 4x1 vector of learned weights
    private final double rMax;

    public ResidualController1DOF(double mNom, double bNom, double gNom,
                                  double Kp, double Kd,
                                  SimpleMatrix w, double rMax) {
        this.mNom = mNom;
        this.bNom = bNom;
        this.gNom = gNom;
        this.Kp = Kp;
        this.Kd = Kd;
        this.w = w;
        this.rMax = rMax;
    }

    private double nominalTorque(double q, double qd,
                                 double qdDes, double qddDes) {
        double e = q - qdDes;
        double edot = qd - qdDes;
        double v = qddDes - Kd * edot - Kp * e;
        return mNom * v + bNom * qd + gNom * Math.sin(q);
    }

    private double saturate(double x) {
        if (x > rMax) return rMax;
        if (x < -rMax) return -rMax;
        return x;
    }

    public double computeTorque(double q, double qd,
                                double qdDes, double qddDes) {
        // z = [q, qd, qdDes, qddDes]^T
        double[] zArr = new double[] { q, qd, qdDes, qddDes };
        SimpleMatrix z = new SimpleMatrix(4, 1, true, zArr);

        double tauNom = nominalTorque(q, qd, qdDes, qddDes);
        double rHat = w.dot(z);
        rHat = saturate(rHat);

        return tauNom + rHat;
    }
}
