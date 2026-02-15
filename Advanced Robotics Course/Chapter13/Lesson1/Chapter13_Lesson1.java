public class SimToRealGap1DOF {

    static class State {
        double theta;
        double omega;
        State(double theta, double omega) {
            this.theta = theta;
            this.omega = omega;
        }
    }

    static double Kp = 5.0;
    static double Kd = 1.0;
    static double dt = 0.02;

    static double policy(State x) {
        return -Kp * x.theta - Kd * x.omega;
    }

    static State step(State x, double u, double c) {
        double thetaNext = x.theta + dt * x.omega;
        double omegaNext = x.omega + dt * (u - c * x.omega);
        return new State(thetaNext, omegaNext);
    }

    static double rollout(double c, int horizon) {
        State x = new State(0.5, 0.0);
        double gamma = 0.99;
        double totalReward = 0.0;
        double powGamma = 1.0;
        for (int t = 0; t < horizon; t++) {
            double u = policy(x);
            double cost = x.theta * x.theta
                          + 0.1 * x.omega * x.omega
                          + 0.01 * u * u;
            totalReward += powGamma * (-cost);
            powGamma *= gamma;
            x = step(x, u, c);
        }
        return totalReward;
    }

    public static void main(String[] args) {
        double Jsim = rollout(0.1, 500);
        double Jreal = rollout(0.3, 500);
        System.out.println("J_sim  = " + Jsim);
        System.out.println("J_real = " + Jreal);
        System.out.println("Gap    = " + (Jreal - Jsim));
    }
}
      
