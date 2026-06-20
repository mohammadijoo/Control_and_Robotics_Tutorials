
import org.ejml.simple.SimpleMatrix;

public class SimpleMPC {
    private static final double Ts = 0.02;
    private static final double I  = 1.0;
    private static final int N = 20;
    private static final int n = 2;
    private static final int m = 1;

    private final SimpleMatrix A;
    private final SimpleMatrix B;
    private final SimpleMatrix H;
    private final SimpleMatrix F;

    public SimpleMPC() {
        A = new SimpleMatrix(new double[][] {
            {1.0, Ts},
            {0.0, 1.0}
        });
        B = new SimpleMatrix(new double[][] {
            {0.5 * Ts * Ts / I},
            {Ts / I}
        });

        SimpleMatrix Q = SimpleMatrix.diag(10.0, 1.0);
        SimpleMatrix R = new SimpleMatrix(1,1);
        R.set(0,0, 0.1);
        SimpleMatrix P = Q.copy();

        SimpleMatrix A_bar = new SimpleMatrix(N * n, n);
        SimpleMatrix B_bar = new SimpleMatrix(N * n, N * m);

        for (int i = 0; i < N; ++i) {
            SimpleMatrix A_power = SimpleMatrix.identity(n);
            for (int k = 0; k < i + 1; ++k) {
                A_power = A_power.mult(A);
            }
            A_bar.insertIntoThis(i*n, 0, A_power);

            for (int j = 0; j <= i; ++j) {
                SimpleMatrix A_ij = SimpleMatrix.identity(n);
                for (int k = 0; k < i - j; ++k) {
                    A_ij = A_ij.mult(A);
                }
                SimpleMatrix block = A_ij.mult(B);
                B_bar.insertIntoThis(i*n, j*m, block);
            }
        }

        SimpleMatrix Q_bar = new SimpleMatrix(N * n, N * n);
        Q_bar.zero();
        for (int i = 0; i < N - 1; ++i) {
            Q_bar.insertIntoThis(i*n, i*n, Q);
        }
        Q_bar.insertIntoThis((N-1)*n, (N-1)*n, P);

        SimpleMatrix R_bar = new SimpleMatrix(N * m, N * m);
        R_bar.zero();
        for (int i = 0; i < N; ++i) {
            R_bar.insertIntoThis(i*m, i*m, R);
        }

        H = B_bar.transpose().mult(Q_bar).mult(B_bar).plus(R_bar);
        F = B_bar.transpose().mult(Q_bar).mult(A_bar);
    }

    public double mpcControl(SimpleMatrix x) {
        SimpleMatrix rhs = F.mult(x);
        SimpleMatrix U_star = H.solve(rhs).scale(-1.0);
        return U_star.get(0);
    }

    public static void main(String[] args) {
        SimpleMPC mpc = new SimpleMPC();
        SimpleMatrix x = new SimpleMatrix(2,1);
        x.set(0, 0, 0.5);
        x.set(1, 0, 0.0);

        int N_steps = (int)(1.0 / Ts);
        for (int k = 0; k < N_steps; ++k) {
            double u = mpc.mpcControl(x);
            x = mpc.A.mult(x).plus(mpc.B.scale(u));
            System.out.println("k=" + k +
                               " q=" + x.get(0) +
                               " qdot=" + x.get(1) +
                               " u=" + u);
        }
    }
}
