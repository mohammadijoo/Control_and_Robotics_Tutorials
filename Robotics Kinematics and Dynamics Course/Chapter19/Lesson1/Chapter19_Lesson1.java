public class OneLinkModel {

    // theta = [I, b, fc, mgL]
    private final double[] theta;

    public OneLinkModel(double[] theta) {
        if (theta.length != 4) {
            throw new IllegalArgumentException("theta must have length 4");
        }
        this.theta = theta.clone();
    }

    private double[] regressor(double q, double qd, double qdd) {
        double sgn;
        if (qd > 0.0) sgn = 1.0;
        else if (qd < 0.0) sgn = -1.0;
        else sgn = 0.0;

        return new double[] { qdd, qd, sgn, Math.sin(q) };
    }

    public double torque(double q, double qd, double qdd) {
        double[] Y = regressor(q, qd, qdd);
        double tau = 0.0;
        for (int i = 0; i < 4; ++i) {
            tau += Y[i] * theta[i];
        }
        return tau;
    }

    public static void main(String[] args) {
        double[] thetaTrue = {0.8, 0.05, 0.2, 3.0};
        double[] thetaNominal = {1.0, 0.02, 0.1, 2.5};

        OneLinkModel modelTrue = new OneLinkModel(thetaTrue);
        OneLinkModel modelNom = new OneLinkModel(thetaNominal);

        double q = 0.5, qd = 0.1, qdd = -0.3;
        double tauTrue = modelTrue.torque(q, qd, qdd);
        double tauNom = modelNom.torque(q, qd, qdd);

        System.out.println("tau_true = " + tauTrue);
        System.out.println("tau_nominal = " + tauNom);
        System.out.println("difference = " + (tauNom - tauTrue));
    }
}
      
