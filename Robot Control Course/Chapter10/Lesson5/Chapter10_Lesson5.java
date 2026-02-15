
import org.ejml.simple.SimpleMatrix;

public class JointMPC {
    private int n = 2;
    private double Ts = 0.02;
    private int N = 20;

    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix Q;
    private SimpleMatrix R;

    public JointMPC() {
        int nx = 2 * n;
        int nu = n;

        SimpleMatrix I = SimpleMatrix.identity(n);
        A = new SimpleMatrix(nx, nx);
        B = new SimpleMatrix(nx, nu);

        A.zero();
        A.insertIntoThis(0, 0, I);
        A.insertIntoThis(0, n, I.scale(Ts));
        A.insertIntoThis(n, n, I);

        B.zero();
        B.insertIntoThis(n, 0, I.scale(Ts));

        Q = new SimpleMatrix(nx, nx);
        Q.zero();
        Q.set(0, 0, 100.0);
        Q.set(1, 1, 100.0);
        Q.set(2, 2, 10.0);
        Q.set(3, 3, 10.0);

        R = SimpleMatrix.identity(nu).scale(0.1);
    }

    public double[] computeControl(double[] x0Array) {
        // For brevity, we use a one-step LQR-like feedback as approximation:
        // u = -K x, where K solves a discrete Riccati equation (can be computed offline).
        // In ojAlgo you would form H and q and solve the QP instead.
        SimpleMatrix x0 = new SimpleMatrix(2 * n, 1, true, x0Array);

        // Placeholder gain (to be computed offline)
        double[] Kdata = { 50.0, 0.0, 10.0, 0.0,
                           0.0, 50.0, 0.0, 10.0 };
        SimpleMatrix K = new SimpleMatrix(n, 2 * n, true, Kdata);

        SimpleMatrix u = K.mult(x0).negative();
        return u.getDDRM().getData();
    }

    public static void main(String[] args) {
        JointMPC mpc = new JointMPC();
        double[] x0 = { 0.2, -0.1, 0.0, 0.0 };
        double[] u0 = mpc.computeControl(x0);
        System.out.println("u0[0] = " + u0[0] + ", u0[1] = " + u0[1]);
    }
}
