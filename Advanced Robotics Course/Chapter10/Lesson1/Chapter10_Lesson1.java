import org.ejml.simple.SimpleMatrix;
import java.util.List;

public class RigidRegistration {

    public static class Point3D {
        public double x, y, z;
        public Point3D(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }
    }

    public static class RigidTransform {
        public SimpleMatrix R; // 3x3
        public SimpleMatrix t; // 3x1
    }

    public static RigidTransform estimate(List<Point3D> P, List<Point3D> Q) {
        int N = P.size();
        if (N != Q.size()) {
            throw new IllegalArgumentException("Point lists must have same size");
        }

        // Compute centroids
        double px = 0, py = 0, pz = 0;
        double qx = 0, qy = 0, qz = 0;
        for (int i = 0; i < N; ++i) {
            px += P.get(i).x; py += P.get(i).y; pz += P.get(i).z;
            qx += Q.get(i).x; qy += Q.get(i).y; qz += Q.get(i).z;
        }
        px /= N; py /= N; pz /= N;
        qx /= N; qy /= N; qz /= N;

        // Build centered matrices (3 x N)
        SimpleMatrix Pm = new SimpleMatrix(3, N);
        SimpleMatrix Qm = new SimpleMatrix(3, N);
        for (int i = 0; i < N; ++i) {
            Point3D p = P.get(i);
            Point3D q = Q.get(i);
            Pm.set(0, i, p.x - px);
            Pm.set(1, i, p.y - py);
            Pm.set(2, i, p.z - pz);
            Qm.set(0, i, q.x - qx);
            Qm.set(1, i, q.y - qy);
            Qm.set(2, i, q.z - qz);
        }

        // Cross-covariance H = Pm * Qm^T
        SimpleMatrix H = Pm.mult(Qm.transpose());

        // SVD of H
        SimpleMatrix U = new SimpleMatrix(3, 3);
        SimpleMatrix W = new SimpleMatrix(3, 3);
        SimpleMatrix Vt = new SimpleMatrix(3, 3);
        H.svd(U, W, Vt);

        SimpleMatrix R = Vt.transpose().mult(U.transpose());

        // Handle reflection if det(R) < 0
        double detR = R.determinant();
        if (detR < 0) {
            // Flip last column of V
            Vt.set(2, 0, -Vt.get(2, 0));
            Vt.set(2, 1, -Vt.get(2, 1));
            Vt.set(2, 2, -Vt.get(2, 2));
            R = Vt.transpose().mult(U.transpose());
        }

        // t = q_bar - R * p_bar
        SimpleMatrix p_bar = new SimpleMatrix(3, 1, true, new double[]{px, py, pz});
        SimpleMatrix q_bar = new SimpleMatrix(3, 1, true, new double[]{qx, qy, qz});
        SimpleMatrix t = q_bar.minus(R.mult(p_bar));

        RigidTransform T = new RigidTransform();
        T.R = R;
        T.t = t;
        return T;
    }
}
      
