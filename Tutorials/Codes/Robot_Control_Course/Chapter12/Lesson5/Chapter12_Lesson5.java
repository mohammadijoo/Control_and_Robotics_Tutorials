
import java.util.concurrent.*;
import java.util.*;
import java.lang.Math;

class JointInterface {
    public double readPosition() { /* ... */ return 0.0; }
    public double readVelocity() { /* ... */ return 0.0; }
    public void writeTorque(double tau) { /* ... */ }
}

public class RealTimeController {
    public static void main(String[] args) {
        final double Ts = 0.002; // 2 ms
        final double Kp = 50.0;
        final double Kd = 2.0;
        final double tFinal = 5.0;

        JointInterface joint = new JointInterface();

        List<Double> log_t = new ArrayList<>();
        List<Double> log_Tk = new ArrayList<>();
        List<Double> log_jitter = new ArrayList<>();
        List<Double> log_q = new ArrayList<>();
        List<Double> log_qd = new ArrayList<>();
        List<Double> log_e = new ArrayList<>();

        ScheduledExecutorService exec = Executors.newSingleThreadScheduledExecutor();

        final long startNano = System.nanoTime();
        final double[] ePrev = new double[]{0.0};
        final long periodNano = (long)(Ts * 1e9);

        Runnable task = new Runnable() {
            long lastNano = startNano;

            @Override
            public void run() {
                long nowNano = System.nanoTime();
                double t = (nowNano - startNano) / 1e9;
                if (t >= tFinal) {
                    exec.shutdown();
                    return;
                }

                double Tk = (nowNano - lastNano) / 1e9;
                lastNano = nowNano;
                double jitter = Tk - Ts;

                double q  = joint.readPosition();
                double dq = joint.readVelocity();

                double qd  = 0.5 * Math.sin(2.0 * Math.PI * 0.5 * t);
                double dqd = 0.5 * 2.0 * Math.PI * 0.5 * Math.cos(2.0 * Math.PI * 0.5 * t);

                double e  = qd - q;
                double de = (e - ePrev[0]) / Math.max(Tk, 1e-6);
                ePrev[0] = e;

                double tau = Kp * e + Kd * de;
                joint.writeTorque(tau);

                log_t.add(t);
                log_Tk.add(Tk);
                log_jitter.add(jitter);
                log_q.add(q);
                log_qd.add(qd);
                log_e.add(e);
            }
        };

        exec.scheduleAtFixedRate(task, 0, periodNano, TimeUnit.NANOSECONDS);
    }
}
