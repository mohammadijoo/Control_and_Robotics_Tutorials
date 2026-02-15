public class DisturbanceRejectionSimulation {

    public static void main(String[] args) {
        // Physical parameters
        double J = 0.01;
        double b = 0.1;

        // PI controller gains
        double Kp = 20.0;
        double Ki = 50.0;

        // Simulation parameters
        double Ts = 0.001;
        double Tfinal = 5.0;
        int N = (int) (Tfinal / Ts);

        double theta = 0.0;
        double omega = 0.0;
        double eInt = 0.0;
        double thetaRef = 0.0;

        double D0 = 1.0;  // step load torque

        for (int k = 0; k < N; ++k) {
            double t = k * Ts;

            double d = D0;

            double e = thetaRef - theta;
            eInt += e * Ts;

            double u = Kp * e + Ki * eInt;

            double thetaDDot = (u - d - b * omega) / J;

            omega += Ts * thetaDDot;
            theta += Ts * omega;

            if (k % 1000 == 0) {
                System.out.printf("t=%.3f  theta=%.5f  omega=%.5f%n",
                        t, theta, omega);
            }
        }

        System.out.printf("Final position (steady-state error) [rad]: %.6f%n",
                theta);
    }
}
