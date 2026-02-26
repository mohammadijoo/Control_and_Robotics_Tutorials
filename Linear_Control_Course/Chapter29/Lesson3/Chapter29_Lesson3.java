public class ThermalPISim {

    public static void main(String[] args) {
        double tauTh = 300.0;  // s
        double Kth   = 5.0;    // K per unit input

        double Kp = 0.05;
        double Ki = 1.0e-4;

        double Ts = 1.0;       // seconds
        int N = 4000;

        double T = 0.0;        // Delta T
        double integ = 0.0;
        double u = 0.0;
        double r = 1.0;        // 1 K step

        for (int k = 0; k < N; k++) {
            double e = r - T;
            integ += e * Ts;
            u = Kp * e + Ki * integ;

            // Saturation
            if (u > 1.0) u = 1.0;
            if (u < 0.0) u = 0.0;

            double dTdt = (-1.0 / tauTh) * T + (Kth / tauTh) * u;
            T += Ts * dTdt;

            if (k % 100 == 0) {
                System.out.printf("t = %.1f s, T = %.3f K, u = %.3f%n",
                                  k * Ts, T, u);
            }
        }
    }
}
