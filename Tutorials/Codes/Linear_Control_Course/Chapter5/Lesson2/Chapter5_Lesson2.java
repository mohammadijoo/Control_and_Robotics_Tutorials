import java.util.ArrayList;
import java.util.List;

public class FirstOrderResponse {
    public static void main(String[] args) {
        double K   = 2.0;
        double tau = 0.5;
        double dt  = 0.001;
        double T   = 5.0;

        int N = (int) (T / dt);
        List<Double> tList = new ArrayList<>(N);
        List<Double> yList = new ArrayList<>(N);

        double y = 0.0; // initial condition

        for (int k = 0; k < N; ++k) {
            double t = k * dt;

            double uStep    = 1.0;           // unit step
            double uRamp    = t;             // unit ramp
            double uImpulse = (k == 0) ? 1.0 / dt : 0.0; // discrete impulse

            double u = uStep; // choose the desired test input

            double dydt = (-y + K * u) / tau;
            y += dt * dydt;

            tList.add(t);
            yList.add(y);
        }

        for (int k = 0; k < N; k += N / 10) {
            System.out.printf("t = %.3f, y = %.4f%n", tList.get(k), yList.get(k));
        }

        // In Java-based robotics (e.g. WPILib), this logic can be placed inside
        // the periodic control loop to simulate plant dynamics or perform model-based control.
    }
}
