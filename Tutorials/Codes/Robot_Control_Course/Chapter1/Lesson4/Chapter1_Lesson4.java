
public class PdStabilityDemo {
    public static void main(String[] args) {
        double I = 1.0;
        double kp = 50.0;
        double kd = 10.0;

        double q = 0.2;     // initial position error
        double dq = 0.0;    // initial velocity
        double dt = 0.001;  // time step
        double T  = 2.0;    // total simulation time

        double t = 0.0;
        while (t < T) {
            // Dynamics: I * ddq + kd * dq + kp * q = 0
            double ddq = -(kd * dq + kp * q) / I;

            // Integrate
            dq += dt * ddq;
            q  += dt * dq;
            t  += dt;

            // Lyapunov function (energy-like)
            double V = 0.5 * I * dq * dq + 0.5 * kp * q * q;
            if ((int)(t * 1000) % 100 == 0) {
                System.out.println("t=" + t + " q=" + q + " dq=" + dq + " V=" + V);
            }
        }
    }
}
