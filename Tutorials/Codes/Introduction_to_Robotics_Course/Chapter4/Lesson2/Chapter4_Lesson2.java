
public class MotorLoopSim {
    public static void main(String[] args) {
        double Ts = 1e-3, Tend = 0.5;
        int steps = (int)(Tend/Ts);

        // Simple first-order motor speed model: omegaDot = (-omega + k*u)/T
        double Tm = 0.05, k = 15.0;
        double omega = 0.0, theta = 0.0;

        int N = 4096;
        double delta = 2*Math.PI/N;

        double Kp = 2.0, Kd = 0.02;
        double ref = 1.0;

        for(int i=0;i<steps;i++){
            // encoder quantization
            double theta_q = delta*Math.round(theta/delta);

            double e = ref - theta_q;
            double u = Kp*e - Kd*omega;
            u = Math.max(-12.0, Math.min(12.0, u));

            double omegaDot = (-omega + k*u)/Tm;
            omega += Ts*omegaDot;
            theta += Ts*omega;
        }

        System.out.println("theta(Tend) = " + theta);
    }
}
      