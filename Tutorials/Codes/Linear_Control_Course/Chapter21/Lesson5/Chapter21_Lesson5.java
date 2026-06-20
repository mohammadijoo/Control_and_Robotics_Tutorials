public class ServoController {
    private final double Kc;
    private final double zi;
    private final double zl;
    private final double pl;
    private final double Ts;

    private double xInt = 0.0;
    private double xLead = 0.0;

    public ServoController(double Kc, double zi, double zl, double pl, double Ts) {
        this.Kc = Kc;
        this.zi = zi;
        this.zl = zl;
        this.pl = pl;
        this.Ts = Ts;
    }

    // One iteration of the control loop
    public double step(double r, double y) {
        double e = r - y;

        // Integrator
        xInt += Ts * zi * e;

        // Lead filter
        double a = 1.0 + pl * Ts;
        double b = zl * Ts;
        double uLead = (b * e + xLead) / a;
        xLead = b * e + (1.0 - pl * Ts) * xLead;

        // Control signal
        return Kc * (e + xInt + uLead);
    }
}
