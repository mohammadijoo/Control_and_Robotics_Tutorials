import org.ejml.simple.SimpleMatrix;

public class RobotPoeJava {
    // S_list: 6 x n matrix (each column is a twist)
    private final SimpleMatrix S_list;
    private final SimpleMatrix M;
    private final int n;

    public RobotPoeJava(SimpleMatrix S_list, SimpleMatrix M) {
        if (S_list.numRows() != 6) {
            throw new IllegalArgumentException("S_list must be 6 x n");
        }
        this.S_list = S_list;
        this.M = M;
        this.n = S_list.numCols();
    }

    private SimpleMatrix skew3(SimpleMatrix w) {
        double wx = w.get(0);
        double wy = w.get(1);
        double wz = w.get(2);
        double[][] data = {
            {0.0,   -wz,   wy},
            {wz,    0.0,  -wx},
            {-wy,   wx,    0.0}
        };
        return new SimpleMatrix(data);
    }

    private SimpleMatrix expTwist(SimpleMatrix xi, double q, double eps) {
        SimpleMatrix w = xi.extractMatrix(0, 3, 0, 1);
        SimpleMatrix v = xi.extractMatrix(3, 6, 0, 1);
        double theta = q;
        double normW = Math.sqrt(
            w.get(0) * w.get(0) +
            w.get(1) * w.get(1) +
            w.get(2) * w.get(2)
        );

        SimpleMatrix T = SimpleMatrix.identity(4);
        if (normW < eps) {
            // prismatic
            T.insertIntoThis(0, 0, SimpleMatrix.identity(3));
            T.insertIntoThis(0, 3, v.scale(theta));
        } else {
            SimpleMatrix w_unit = w.divide(normW);
            SimpleMatrix W = skew3(w_unit);
            SimpleMatrix W2 = W.mult(W);
            SimpleMatrix I3 = SimpleMatrix.identity(3);

            SimpleMatrix R = I3
                .plus(W.scale(Math.sin(theta)))
                .plus(W2.scale(1.0 - Math.cos(theta)));

            SimpleMatrix V = I3.scale(theta)
                .plus(W.scale(1.0 - Math.cos(theta)))
                .plus(W2.scale(theta - Math.sin(theta)));

            T.insertIntoThis(0, 0, R);
            T.insertIntoThis(0, 3, V.mult(v));
        }
        return T;
    }

    public SimpleMatrix fk(double[] q) {
        if (q.length != n) {
            throw new IllegalArgumentException("q must have length n");
        }
        SimpleMatrix T = SimpleMatrix.identity(4);
        double eps = 1e-9;
        for (int i = 0; i < n; ++i) {
            SimpleMatrix xi = S_list.extractMatrix(0, 6, i, i + 1);
            T = T.mult(expTwist(xi, q[i], eps));
        }
        return T.mult(M);
    }
}
      
