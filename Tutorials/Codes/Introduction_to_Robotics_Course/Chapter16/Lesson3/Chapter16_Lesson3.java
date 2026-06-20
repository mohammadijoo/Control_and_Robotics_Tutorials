public class CoDesignServo {
    public static void main(String[] args) {
        double J  = 0.02;
        double B  = 0.01;
        double N  = 10.0;
        double Kt = 0.05;
        double Ke = 0.05;
        double R  = 2.0;
        double Kp = 32.0;
        double Kd = 1.7;
        double qRef = 0.5;

        double Beq = B + (N * N) * Kt * Ke / R;
        double Ka  = N * Kt / R;

        double dt = 0.0005;
        int steps = 20000;
        double q  = 0.0;
        double qd = 0.0;

        for (int k = 0; k < steps; ++k) {
            double e   = qRef - q;
            double v   = Kp * e - Kd * qd;
            double tau = Ka * v - Beq * qd;
            double qdd = tau / J;

            qd += dt * qdd;
            q  += dt * qd;

            if (k % 4000 == 0) {
                System.out.println((k * dt) + "  " + q);
            }
        }
    }
}
      
