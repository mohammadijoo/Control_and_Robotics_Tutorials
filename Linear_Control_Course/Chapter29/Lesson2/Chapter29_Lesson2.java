public class LinearAxisServo {

    public static void main(String[] args) {
        double J_eq = 0.01;
        double b_eq = 0.001;
        double K_t  = 0.1;
        double K_c  = 2.0;
        double N    = 50.0;

        double Kv    = (K_t * K_c) / (N * b_eq);
        double tau_m = J_eq / b_eq;

        double dt   = 0.0005;
        int    Nsim = 4000;

        double Kp = 50.0;
        double Ki = 500.0;
        double Kd = 0.001;

        double theta     = 0.0;
        double thetaDot  = 0.0;
        double ref       = 0.1;
        double e         = 0.0;
        double ePrev     = 0.0;
        double I         = 0.0;

        for (int k = 0; k < Nsim; ++k) {
            double t = k * dt;

            e = ref - theta;
            I += e * dt;
            double D = (e - ePrev) / dt;
            double u = Kp * e + Ki * I + Kd * D;

            double thetaDotDot = -(b_eq / J_eq) * thetaDot + (Kv / J_eq) * u;
            thetaDot += thetaDotDot * dt;
            theta    += thetaDot * dt;

            ePrev = e;

            if (k % 200 == 0) {
                System.out.printf("t=%.4f theta=%.4f u=%.4f%n", t, theta, u);
            }
        }
    }
}
