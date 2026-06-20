public class CascadeController {
    private double k1_p; // inner gain (velocity)
    private double k2_p; // outer gain (position)
    private double dt;

    private double v;    // velocity state
    private double x;    // position state

    public CascadeController(double k1_p, double k2_p, double dt) {
        this.k1_p = k1_p;
        this.k2_p = k2_p;
        this.dt = dt;
        this.v = 0.0;
        this.x = 0.0;
    }

    // One simulation step given desired position rPos
    public void step(double rPos) {
        double ePos = rPos - x;
        double vRef = k2_p * ePos;

        double eVel = vRef - v;
        double u = k1_p * eVel;

        // Simple plant model: dv/dt = -alpha * v + beta * u
        double alpha = 50.0;
        double beta = 50.0;
        double dv = -alpha * v + beta * u;

        v += dv * dt;
        x += v * dt;
    }

    public double getPosition() {
        return x;
    }

    public double getVelocity() {
        return v;
    }

    public static void main(String[] args) {
        CascadeController ctrl = new CascadeController(15.0, 4.0, 0.001);
        double rPos = 1.0;

        for (int k = 0; k < 5000; ++k) {
            ctrl.step(rPos);
            if (k % 1000 == 0) {
                System.out.println("t = " + (k * 0.001)
                        + "  x = " + ctrl.getPosition()
                        + "  v = " + ctrl.getVelocity());
            }
        }
    }
}
