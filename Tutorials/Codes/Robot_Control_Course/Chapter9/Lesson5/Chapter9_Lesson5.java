
import org.ejml.simple.SimpleMatrix;

public class SingleLinkLQR {
    static final double m = 1.0;
    static final double ell = 1.0;
    static final double I = m * ell * ell;
    static final double b = 0.1;
    static final double g = 9.81;
    static final double dt = 0.01;

    static SimpleMatrix dynamics(SimpleMatrix x, double u) {
        double q = x.get(0);
        double dq = x.get(1);
        double ddq = (u - b * dq - m * g * ell * Math.sin(q)) / I;
        return new SimpleMatrix(2, 1, true, new double[]{dq, ddq});
    }

    static SimpleMatrix step(SimpleMatrix x, double u) {
        return x.plus(dynamics(x, u).scale(dt));
    }

    public static void main(String[] args) {
        double qStar = 0.0;
        double dqStar = 0.0;
        double uStar = m * g * ell * Math.sin(qStar);
        SimpleMatrix xStar = new SimpleMatrix(2, 1, true, new double[]{qStar, dqStar});

        // Linearization
        SimpleMatrix A_c = new SimpleMatrix(2, 2, true, new double[]{
            0.0, 1.0,
            -(m * g * ell / I) * Math.cos(qStar), -b / I
        });
        SimpleMatrix B_c = new SimpleMatrix(2, 1, true, new double[]{0.0, 1.0 / I});
        SimpleMatrix A_d = SimpleMatrix.identity(2).plus(A_c.scale(dt));
        SimpleMatrix B_d = B_c.scale(dt);

        SimpleMatrix Q = new SimpleMatrix(2, 2, true, new double[]{
            10.0, 0.0,
            0.0, 1.0
        });
        SimpleMatrix Qf = new SimpleMatrix(2, 2, true, new double[]{
            50.0, 0.0,
            0.0, 5.0
        });
        SimpleMatrix R = new SimpleMatrix(1, 1, true, new double[]{0.1});

        int N = 500;
        SimpleMatrix[] P = new SimpleMatrix[N + 1];
        SimpleMatrix[] K = new SimpleMatrix[N];

        P[N] = Qf.copy();
        for (int k = N - 1; k >= 0; --k) {
            SimpleMatrix S = R.plus(B_d.transpose().mult(P[k + 1]).mult(B_d));
            SimpleMatrix Kk = S.solve(B_d.transpose().mult(P[k + 1]).mult(A_d)).negative();
            K[k] = Kk;
            SimpleMatrix Acl = A_d.plus(B_d.mult(Kk));
            P[k] = Q.plus(Acl.transpose().mult(P[k + 1]).mult(Acl));
        }

        // Simulate LQR stabilization
        SimpleMatrix x = new SimpleMatrix(2, 1, true, new double[]{0.5, 0.0});
        for (int k = 0; k < 500; ++k) {
            SimpleMatrix dx = x.minus(xStar);
            SimpleMatrix Kk = K[Math.min(k, N - 1)];
            double u = uStar + Kk.mult(dx).get(0);
            x = step(x, u);
            System.out.println(k * dt + " " + x.get(0) + " " + x.get(1) + " " + u);
        }
    }
}
