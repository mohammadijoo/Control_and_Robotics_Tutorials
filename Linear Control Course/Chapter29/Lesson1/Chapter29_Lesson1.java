public class DCMotor {
    double R = 2.0;
    double L = 0.5;
    double J = 0.02;
    double b = 0.002;
    double Kt = 0.1;
    double Ke = 0.1;

    double ia = 0.0;
    double omega = 0.0;
    double theta = 0.0;

    public void step(double va, double TL, double dt) {
        double dia = (va - R * ia - Ke * omega) / L;
        double domega = (Kt * ia - b * omega - TL) / J;

        ia += dia * dt;
        omega += domega * dt;
        theta += omega * dt;
    }
}

class PIController {
    double Kp;
    double Ki;
    double integ;

    public PIController(double Kp, double Ki) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.integ = 0.0;
    }

    public double control(double ref, double meas, double dt) {
        double e = ref - meas;
        integ += e * dt;
        return Kp * e + Ki * integ;
    }
}

public class DCMotorSimulation {
    public static void main(String[] args) {
        DCMotor motor = new DCMotor();
        PIController pi = new PIController(1.5, 50.0);

        double dt = 0.0005;
        double tEnd = 1.0;
        int steps = (int) (tEnd / dt);
        double TL = 0.0;

        System.out.println("t,omega,theta");

        for (int k = 0; k <= steps; k++) {
            double t = k * dt;
            double omegaRef = (t >= 0.0) ? 1.0 : 0.0;

            double va = pi.control(omegaRef, motor.omega, dt);
            motor.step(va, TL, dt);

            if (k % 50 == 0) {
                System.out.println(t + "," + motor.omega + "," + motor.theta);
            }
        }
    }
}
