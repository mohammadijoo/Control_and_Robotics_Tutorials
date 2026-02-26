import java.util.Random;

public class JointUncertaintySweep {

    static class State {
        double theta;
        double thetaDot;
    }

    static State[] simulate(double J, double b,
                            double u, double dt, double T) {
        int N = (int) (T / dt);
        State[] traj = new State[N + 1];
        State x = new State();
        x.theta = 0.0;
        x.thetaDot = 0.0;
        traj[0] = x;

        for (int k = 0; k < N; ++k) {
            State xnew = new State();
            double thetaDDot = (u - b * x.thetaDot) / J;
            xnew.thetaDot = x.thetaDot + dt * thetaDDot;
            xnew.theta    = x.theta + dt * xnew.thetaDot;
            traj[k + 1] = xnew;
            x = xnew;
        }
        return traj;
    }

    public static void main(String[] args) {
        double J0 = 0.01;
        double b0 = 0.05;
        double u  = 1.0;
        double dt = 0.001;
        double T  = 4.0;

        Random rng = new Random(1L);
        for (int i = 0; i < 5; ++i) {
            double deltaJ = 0.4 * (rng.nextDouble() - 0.5);
            double deltaB = 0.4 * (rng.nextDouble() - 0.5);
            double J = J0 * (1.0 + deltaJ);
            double b = b0 * (1.0 + deltaB);

            State[] traj = simulate(J, b, u, dt, T);
            State xf = traj[traj.length - 1];
            System.out.printf(
                "Sample %d: J=%.4f, b=%.4f, final theta=%.4f%n",
                i, J, b, xf.theta
            );
        }
    }
}
