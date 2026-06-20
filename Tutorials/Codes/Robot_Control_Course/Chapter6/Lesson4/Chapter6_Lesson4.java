
public class Admittance1D {
    private final double M_a;
    private final double D_a;
    private final double K_a;
    private final double x0;
    private final double dt;

    private double z1;
    private double z2;

    private final double A11, A12, A21, A22, B1, B2;

    public Admittance1D(double M_a, double D_a, double K_a, double x0, double dt) {
        this.M_a = M_a;
        this.D_a = D_a;
        this.K_a = K_a;
        this.x0 = x0;
        this.dt = dt;

        this.z1 = 0.0;
        this.z2 = 0.0;

        double alpha = dt / M_a;
        this.A11 = 1.0 - dt * alpha * K_a;
        this.A12 = dt * (1.0 - alpha * D_a);
        this.A21 = -alpha * K_a;
        this.A22 = 1.0 - alpha * D_a;
        this.B1 = dt * alpha;
        this.B2 = alpha;
    }

    public double[] step(double F_ext) {
        double z1Next = A11 * z1 + A12 * z2 + B1 * F_ext;
        double z2Next = A21 * z1 + A22 * z2 + B2 * F_ext;

        z1 = z1Next;
        z2 = z2Next;

        double x_r = z1 + x0;
        double xdot_r = z2;
        return new double[]{x_r, xdot_r};
    }

    public static void main(String[] args) {
        Admittance1D ctrl = new Admittance1D(3.0, 20.0, 50.0, 0.0, 0.001);
        double F_ext = 10.0;
        for (int k = 0; k < 2000; ++k) {
            double[] state = ctrl.step(F_ext);
            if (k % 500 == 0) {
                System.out.println("k=" + k
                        + ", x_r=" + state[0]
                        + ", xdot_r=" + state[1]);
            }
        }
    }
}
