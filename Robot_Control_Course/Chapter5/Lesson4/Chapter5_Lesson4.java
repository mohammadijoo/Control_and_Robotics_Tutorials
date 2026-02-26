
public class OneDofImpactController {

    public enum Mode { FREE, CONTACT }

    private final double m;
    private final double g;
    private final double e;
    private Mode mode;

    public OneDofImpactController(double m, double g, double e) {
        this.m = m;
        this.g = g;
        this.e = e;
        this.mode = Mode.FREE;
    }

    public static class State {
        public double q;
        public double v;
    }

    public void step(double t, double dt, State x, double qDes) {
        // detect events
        if (mode == Mode.FREE && x.q <= 0.0 && x.v < 0.0) {
            // impact
            x.v = -e * x.v;
            x.q = 0.0;
            mode = Mode.CONTACT;
        } else if (mode == Mode.CONTACT && x.v >= 0.0 && x.q >= 0.0) {
            // liftoff
            mode = Mode.FREE;
        }

        double tau;
        if (mode == Mode.FREE) {
            tau = pdControl(x.q, x.v, qDes, 0.0);
        } else {
            // contact mode: hold at q = 0 with high damping
            tau = 0.0;
        }

        // integrate with explicit Euler
        double a = (tau - m * g) / m;
        x.q = x.q + dt * x.v;
        x.v = x.v + dt * a;
    }

    private double pdControl(double q, double v, double qDes, double vDes) {
        double kp = 50.0;
        double kd = 10.0;
        double tauG = m * g;
        double tauPD = -kp * (q - qDes) - kd * (v - vDes);
        return tauG + tauPD;
    }
}
