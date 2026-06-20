public class PIController {
    private final double Kp;
    private final double Ki;
    private final double dt;

    private double integrator = 0.0;
    private double prevError = 0.0;

    public PIController(double Kp, double Ki, double dt) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.dt = dt;
    }

    public double update(double error) {
        integrator += 0.5 * (error + prevError) * dt;
        prevError = error;
        return Kp * error + Ki * integrator;
    }
}

class FirstOrderPlant {
    private final double T;
    private final double K;
    private double x = 0.0;

    public FirstOrderPlant(double T, double K) {
        this.T = T;
        this.K = K;
    }

    public double step(double u, double dt) {
        double dx = (-x / T) + (K / T) * u;
        x += dt * dx;
        return x;
    }
}

public class LowLevelLoop {
    public static void main(String[] args) {
        double dt = 0.01;
        PIController controller = new PIController(3.0, 20.0, dt);
        FirstOrderPlant plant = new FirstOrderPlant(0.1, 1.0);

        double r = 1.0; // reference from high-level controller
        double y = 0.0;

        for (int k = 0; k < 2000; ++k) {
            double error = r - y;
            double u = controller.update(error);
            y = plant.step(u, dt);

            // logging or telemetry to higher layer
        }
    }
}
