
public class Interaction1DOF {

    public enum Mode { IMPEDANCE, ADMITTANCE }

    public static class State {
        public double x;
        public double v;
        public double xCommand;
        public double vVirtual;
    }

    // Parameters
    private final double Mr = 5.0;
    private final double Dr = 2.0;
    private final double Ke = 500.0;
    private final double De = 10.0;

    private final double Md = 2.0;
    private final double Dd = 30.0;
    private final double Kd = 200.0;

    public void step(State s, double xRef, double dt, Mode mode) {
        double Fenv = Ke * s.x + De * s.v;
        double Fu;

        if (mode == Mode.IMPEDANCE) {
            double e  = xRef - s.x;
            double ed = 0.0 - s.v;
            Fu = Kd * e + Dd * ed;

            double a = (Fu + Fenv - Dr * s.v) / Mr;
            s.v += dt * a;
            s.x += dt * s.v;

        } else {
            // Admittance
            double ac = (Fenv - Dd * s.vVirtual - Kd * (s.xCommand - xRef)) / Md;
            s.vVirtual += dt * ac;
            s.xCommand += dt * s.vVirtual;

            double ePos = s.xCommand - s.x;
            Fu = 1000.0 * ePos; // inner position loop

            double a = (Fu + Fenv - Dr * s.v) / Mr;
            s.v += dt * a;
            s.x += dt * s.v;
        }
    }
}
