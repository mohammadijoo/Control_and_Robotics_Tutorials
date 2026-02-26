public class InnerPiCurrentLoop {

    private final double kp;
    private final double ki;
    private final double dt;
    private double integral;

    public InnerPiCurrentLoop(double kp, double ki, double dt) {
        this.kp = kp;
        this.ki = ki;
        this.dt = dt;
        this.integral = 0.0;
    }

    public double update(double iRef, double iMeas) {
        double e = iRef - iMeas;
        integral += e * dt;
        return kp * e + ki * integral;
    }

    public static void main(String[] args) {
        // RL parameters
        double L = 2e-3;
        double R = 0.5;
        double Ku = 1.0;

        double kp = 1.4;    // example PI gains
        double ki = 500.0;
        double dt = 1e-4;

        InnerPiCurrentLoop controller = new InnerPiCurrentLoop(kp, ki, dt);

        double i = 0.0;
        double iRef = 5.0;

        int steps = 2000;
        for (int k = 0; k < steps; ++k) {
            double u = controller.update(iRef, i);

            // RL model: di/dt = (-R * i + Ku * u) / L
            double di_dt = (-R * i + Ku * u) / L;
            i += dt * di_dt;

            if (k % 100 == 0) {
                double t = k * dt;
                System.out.println(t + " " + i);
            }
        }
    }
}
