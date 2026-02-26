public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double integral;
    private double prevError;
    private boolean firstCall;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.integral = 0.0;
        this.prevError = 0.0;
        this.firstCall = true;
    }

    public double update(double ref, double y, double dt) {
        double e = ref - y;
        integral += e * dt;

        double derivative = 0.0;
        if (!firstCall) {
            derivative = (e - prevError) / dt;
        } else {
            firstCall = false;
        }
        prevError = e;

        return Kp * e + Ki * integral + Kd * derivative;
    }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        firstCall = true;
    }
}

// Example usage in a main program (simulation skeleton)
public class Example {
    public static void main(String[] args) {
        double Kp = 43.47;
        double Ki = 91.76;
        double Kd = 11.0;

        PIDController pid = new PIDController(Kp, Ki, Kd);

        double dt = 0.001;
        double T_end = 10.0;
        int N = (int)(T_end / dt);

        double x1 = 0.0;
        double x2 = 0.0;
        double y  = 0.0;

        for (int k = 0; k < N; ++k) {
            double t = k * dt;
            double ref = 1.0; // step

            double u = pid.update(ref, y, dt);

            // simple plant approximation: G(s) ~ 1/(s*(s+1))
            double x1_dot = x2;
            double x2_dot = -x2 + u;
            x1 += x1_dot * dt;
            x2 += x2_dot * dt;
            y = x1;

            if (k % 1000 == 0) {
                System.out.println("t=" + t + " y=" + y);
            }
        }
    }
}
