public class PIController {
    private double Kp;
    private double Ki;
    private double integral;

    public PIController(double Kp, double Ki) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.integral = 0.0;
    }

    public double update(double r, double y, double dt) {
        double e = r - y;
        integral += e * dt;
        return Kp * e + Ki * integral;
    }
}

public class JointSimulation {
    // y_dot = -3 y + 5 u
    private double y;

    public JointSimulation() {
        this.y = 0.0;
    }

    public void step(double u, double dt) {
        double yDot = -3.0 * y + 5.0 * u;
        y += dt * yDot;
    }

    public double getY() {
        return y;
    }

    public static void main(String[] args) {
        double dt = 0.001;
        double T  = 5.0;
        int N = (int) (T / dt);

        PIController controller = new PIController(1.0, 4.0);
        JointSimulation joint = new JointSimulation();

        double r = 1.0; // unit step

        for (int k = 0; k < N; ++k) {
            double u = controller.update(r, joint.getY(), dt);
            joint.step(u, dt);
        }

        double eFinal = r - joint.getY();
        System.out.println("Approximate steady-state error: " + eFinal);
    }
}
