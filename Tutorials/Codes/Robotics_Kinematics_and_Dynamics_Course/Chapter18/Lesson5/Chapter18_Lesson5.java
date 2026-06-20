import org.ejml.simple.SimpleMatrix;
import java.util.ArrayList;
import java.util.List;

public class Planar2RDiff {

    static SimpleMatrix fk(SimpleMatrix q, double l1, double l2) {
        double q1 = q.get(0);
        double q2 = q.get(1);
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        return new SimpleMatrix(2, 1, true, x, y);
    }

    static SimpleMatrix jacobian(SimpleMatrix q, double l1, double l2) {
        double q1 = q.get(0);
        double q2 = q.get(1);
        double s1 = Math.sin(q1);
        double c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[] data = {
            -l1 * s1 - l2 * s12, -l2 * s12,
             l1 * c1 + l2 * c12,  l2 * c12
        };
        return new SimpleMatrix(2, 2, true, data);
    }

    static List<SimpleMatrix> centralDiff(List<SimpleMatrix> samples, double h) {
        int N = samples.size();
        List<SimpleMatrix> deriv = new ArrayList<>(N);
        for (int k = 0; k < N; ++k) {
            deriv.add(new SimpleMatrix(samples.get(0).numRows(), 1));
        }
        if (N < 2) {
            return deriv;
        }
        deriv.set(0, new SimpleMatrix(samples.get(0).numRows(), 1));
        deriv.set(N - 1, new SimpleMatrix(samples.get(0).numRows(), 1));
        for (int k = 1; k + 1 < N; ++k) {
            SimpleMatrix num = samples.get(k + 1).minus(samples.get(k - 1));
            deriv.set(k, num.divide(2.0 * h));
        }
        return deriv;
    }

    public static void main(String[] args) {
        double l1 = 1.0;
        double l2 = 0.7;
        double omega = 1.5;
        double T = 5.0;
        int N = 501;
        double h = T / (N - 1.0);

        List<SimpleMatrix> q = new ArrayList<>(N);
        List<SimpleMatrix> dqAna = new ArrayList<>(N);
        List<SimpleMatrix> x = new ArrayList<>(N);

        for (int k = 0; k < N; ++k) {
            double t = k * h;
            SimpleMatrix qk = new SimpleMatrix(2, 1, true,
                    Math.sin(omega * t),
                    Math.cos(omega * t));
            q.add(qk);

            SimpleMatrix dqk = new SimpleMatrix(2, 1, true,
                    omega * Math.cos(omega * t),
                    -omega * Math.sin(omega * t));
            dqAna.add(dqk);

            x.add(fk(qk, l1, l2));
        }

        List<SimpleMatrix> xdNum = centralDiff(x, h);
        List<SimpleMatrix> xdModel = new ArrayList<>(N);
        for (int k = 0; k < N; ++k) {
            SimpleMatrix J = jacobian(q.get(k), l1, l2);
            xdModel.add(J.mult(dqAna.get(k)));
        }

        // Comparison of xdNum vs xdModel can be added here.
    }
}
      
