public class CruiseControlSimulation {

    static class Vehicle {
        double m;   // mass
        double b;   // drag coefficient
        double Ku;  // engine gain
        double v;   // speed state

        Vehicle(double m, double b, double Ku) {
            this.m = m;
            this.b = b;
            this.Ku = Ku;
            this.v = 0.0;
        }

        // One Euler step: dv/dt = (Ku*u - b*v)/m
        void step(double u, double h) {
            double dv = (Ku * u - b * v) / m;
            v += dv * h;
        }
    }

    static class PIController {
        double Kp;
        double Ki;
        double ei;  // integral of error

        PIController(double Kp, double Ki) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.ei = 0.0;
        }

        double compute(double r, double y, double h) {
            double e = r - y;
            ei += e * h;
            return Kp * e + Ki * ei;
        }
    }

    public static void main(String[] args) {
        double m  = 1000.0;
        double b  = 50.0;
        double Ku = 500.0;

        // Example gains
        double Kp = 0.8;
        double Ki = 0.4;

        Vehicle veh = new Vehicle(m, b, Ku);
        PIController ctrl = new PIController(Kp, Ki);

        double h = 0.01;
        double T = 20.0;
        int N = (int) (T / h);

        double v_ref = 10.0; // desired speed [m/s]

        for (int k = 0; k <= N; ++k) {
            double t = k * h;
            double u = ctrl.compute(v_ref, veh.v, h);
            veh.step(u, h);

            if (k % 200 == 0) {
                System.out.println("t = " + t + " s, v = " + veh.v + " m/s");
            }
        }
    }
}
