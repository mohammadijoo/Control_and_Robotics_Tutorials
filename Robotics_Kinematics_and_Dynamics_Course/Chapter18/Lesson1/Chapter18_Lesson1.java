public class Planar2R {
    private final double l1;
    private final double l2;

    public Planar2R(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    public double[] fk(double q1, double q2) {
        double q12 = q1 + q2;
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q12);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q12);
        return new double[] { x, y };
    }

    public static void main(String[] args) {
        Planar2R robot = new Planar2R(1.0, 0.8);

        double T = 2.0;
        int N = 200;
        double dt = T / (double) N;

        double q10 = 0.0, q20 = 0.5;
        double a1 = 0.8, a2 = -0.3;
        double b1 = -0.2, b2 = 0.15;

        double[] t = new double[N + 1];
        double[][] traj = new double[N + 1][2]; // [k][0]=x, [k][1]=y

        for (int k = 0; k <= N; ++k) {
            double tk = k * dt;
            t[k] = tk;

            double q1 = q10 + a1 * tk + b1 * tk * tk;
            double q2 = q20 + a2 * tk + b2 * tk * tk;

            double[] p = robot.fk(q1, q2);
            traj[k][0] = p[0];
            traj[k][1] = p[1];
        }

        // traj now contains the task-space samples
    }
}
      
