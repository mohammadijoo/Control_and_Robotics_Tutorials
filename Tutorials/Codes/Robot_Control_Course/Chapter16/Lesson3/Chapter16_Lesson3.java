
import java.util.concurrent.*;
import java.util.logging.*;
import org.ejml.simple.SimpleMatrix;

public class JointPDController {

    private static final Logger logger = Logger.getLogger("JointPD");

    private final double Ts = 0.001;
    private final double J = 0.05;
    private final double b = 0.01;
    private final double k_p = 50.0;
    private final double k_d = 2.0;
    private final double q_ref = 1.0;

    private final SimpleMatrix A_d;

    public JointPDController() {
        // 2x2 matrix
        double[][] Adata = {
            {1.0, Ts},
            {-Ts * k_p / J, 1.0 - Ts * (k_d + b) / J}
        };
        A_d = new SimpleMatrix(Adata);
    }

    private static class State {
        public double q;
        public double dq;
    }

    private State readState() {
        // TODO: connect to robot or simulator
        State s = new State();
        s.q = 0.0;
        s.dq = 0.0;
        return s;
    }

    private void sendTorque(double u) {
        // TODO: send to hardware or simulator
    }

    public void run(double Tfinal) {
        int N = (int) (Tfinal / Ts);
        ScheduledExecutorService exec = Executors.newSingleThreadScheduledExecutor();

        final double[] e = new double[]{0.0, 0.0}; // position, velocity errors

        Runnable task = new Runnable() {
            int k = 0;
            @Override
            public void run() {
                if (k >= N) {
                    exec.shutdown();
                    return;
                }
                double t_k = k * Ts;
                State s = readState();
                e[0] = s.q - q_ref;
                e[1] = s.dq;

                double u = -k_p * e[0] - k_d * e[1];
                double u_min = -5.0, u_max = 5.0;
                if (u < u_min) u = u_min;
                if (u > u_max) u = u_max;

                sendTorque(u);

                logger.info(String.format(
                    "t=%.4f q=%.3f dq=%.3f e=(%.3f,%.3f) u=%.3f",
                    t_k, s.q, s.dq, e[0], e[1], u
                ));

                k++;
            }
        };

        long periodNs = (long) (Ts * 1e9);
        exec.scheduleAtFixedRate(task, 0, periodNs, TimeUnit.NANOSECONDS);
    }

    public static void main(String[] args) {
        JointPDController ctrl = new JointPDController();
        ctrl.run(5.0);
    }
}
