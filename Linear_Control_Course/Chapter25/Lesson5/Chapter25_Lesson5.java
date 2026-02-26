public class CascadeServo {

    static class PIController {
        double kp, ki, xi, Ts;
        PIController(double kp, double ki, double Ts) {
            this.kp = kp;
            this.ki = ki;
            this.Ts = Ts;
            this.xi = 0.0;
        }
        double update(double ref, double meas) {
            double e = ref - meas;
            xi += e * Ts;
            return kp * e + ki * xi;
        }
    }

    public static void main(String[] args) {
        double Ts = 1e-4;
        int N = 2000;

        PIController currentLoop = new PIController(50.0, 5000.0, Ts);
        PIController speedLoop = new PIController(2.0, 200.0, Ts);
        PIController positionLoop = new PIController(20.0, 200.0, Ts);

        double i = 0.0;
        double w = 0.0;
        double theta = 0.0;

        double J = 0.01;
        double B = 0.001;
        double R = 0.5;
        double L = 1e-3;
        double Kt = 0.05;
        double Ke = 0.05;

        double thetaRef = 1.57; // rad

        for (int k = 0; k < N; ++k) {
            double wRef = positionLoop.update(thetaRef, theta);
            double iRef = speedLoop.update(wRef, w);
            double v = currentLoop.update(iRef, i);

            double di = (-R * i + v - Ke * w) / L;
            double dw = (-B * w + Kt * i) / J;
            double dtheta = w;

            i += di * Ts;
            w += dw * Ts;
            theta += dtheta * Ts;
        }

        System.out.println("Final theta [rad]: " + theta);
    }
}
