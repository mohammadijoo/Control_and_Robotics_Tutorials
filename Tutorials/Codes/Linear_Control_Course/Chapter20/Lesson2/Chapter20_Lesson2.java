public class TwoDofPid {
    private double Kp;
    private double Ki;
    private double Kd;
    private double b;
    private double c;

    private double xi;       // integrator state
    private double eDPrev;   // previous derivative pseudo-error

    public TwoDofPid(double Kp, double Ki, double Kd,
                     double b, double c) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.b = b;
        this.c = c;
        this.xi = 0.0;
        this.eDPrev = 0.0;
    }

    public double step(double r, double y, double dt) {
        double eP = b * r - y;
        double eI = r - y;
        double eD = c * r - y;

        xi = xi + eI * dt;

        double dED = (eD - eDPrev) / dt;
        eDPrev = eD;

        double u = Kp * eP + Ki * xi + Kd * dED;
        return u;
    }

    public static void main(String[] args) {
        TwoDofPid pid = new TwoDofPid(2.0, 5.0, 0.1, 0.6, 0.0);

        double dt = 0.001;
        int nSteps = 5000;

        double y = 0.0;
        double u = 0.0;
        double r = 1.0;
        double T = 0.5;

        for (int k = 0; k != nSteps; ++k) {
            u = pid.step(r, y, dt);

            // Simple plant model; replace with real actuator dynamics
            double dy = (-(1.0 / T) * y + (1.0 / T) * u) * dt;
            y = y + dy;
        }

        // In practice, log y and u or visualize them using a plotting library.
    }
}
