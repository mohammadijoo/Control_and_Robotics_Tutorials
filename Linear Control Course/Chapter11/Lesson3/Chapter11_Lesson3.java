public class PDController {
    private double Kp;
    private double Kd;
    private double dt;
    private double prevError = 0.0;

    public PDController(double Kp, double Kd, double dt) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.dt = dt;
    }

    public double compute(double reference, double measurement) {
        double error = reference - measurement;
        double dedt = (error - prevError) / dt;
        prevError = error;
        return Kp * error + Kd * dedt;
    }
}

// Example usage in a periodic control loop:
public class JointController {
    private final PDController pd;
    private double position = 0.0;
    private double velocity = 0.0;

    private final double M = 1.0;
    private final double B = 0.5;
    private final double K = 4.0;
    private final double dt = 0.001;

    public JointController() {
        pd = new PDController(16.0, 4.0, dt);
    }

    public void update(double reference) {
        double u = pd.compute(reference, position);

        // Simulate mass-spring-damper
        double accel = (u - B * velocity - K * position) / M;
        velocity += accel * dt;
        position += velocity * dt;
    }

    public double getPosition() {
        return position;
    }
}
