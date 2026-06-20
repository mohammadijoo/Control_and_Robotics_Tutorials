public final class JointFriction {

    private JointFriction() {}

    public static double signSmooth(double v, double eps) {
        return Math.tanh(v / eps);
    }

    public static double coulomb(double qdot, double Fc, double eps) {
        return Fc * signSmooth(qdot, eps);
    }

    public static double viscous(double qdot, double b) {
        return b * qdot;
    }

    public static double stribeck(double qdot,
                                  double Fc, double Fs, double vs,
                                  double alpha, double b, double eps) {
        double sgn = signSmooth(qdot, eps);
        double absV = Math.abs(qdot);
        double phi = Math.exp(-Math.pow(absV / vs, alpha));
        return (Fc + (Fs - Fc) * phi) * sgn + b * qdot;
    }

    public static class Params {
        public double Fc = 0.2;
        public double Fs = 0.3;
        public double vs = 0.05;
        public double alpha = 1.0;
        public double b = 0.01;
        public double I = 0.1;
    }

    public static void eulerStep(double[] state, double tauAct,
                                 double dt, Params p) {
        double q = state[0];
        double qdot = state[1];
        double tauF = stribeck(qdot, p.Fc, p.Fs, p.vs, p.alpha, p.b, 1e-3);
        double qddot = (tauAct - tauF) / p.I;
        qdot += dt * qddot;
        q += dt * qdot;
        state[0] = q;
        state[1] = qdot;
    }
}
      
