import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import java.util.ArrayList;
import java.util.List;

class Contact2D {
    public double x, y;
    public double nx, ny;
    public Contact2D(double x, double y, double nx, double ny) {
        this.x = x; this.y = y; this.nx = nx; this.ny = ny;
    }
}

public class ForceClosure2D {

    public static List<double[]> frictionDirections2D(double nx, double ny, double mu) {
        List<double[]> dirs = new ArrayList<>();
        double tx = -ny;
        double ty = nx;
        double phi = Math.atan(mu);
        double cos = Math.cos(phi);
        double sin = Math.sin(phi);
        // d1
        dirs.add(new double[]{cos * nx + sin * tx, cos * ny + sin * ty});
        // d2
        dirs.add(new double[]{cos * nx - sin * tx, cos * ny - sin * ty});
        return dirs;
    }

    public static DMatrixRMaj buildWrenchMatrix2D(
            List<Contact2D> contacts, double mu)
    {
        List<double[]> cols = new ArrayList<>();
        for (Contact2D c : contacts) {
            for (double[] d : frictionDirections2D(c.nx, c.ny, mu)) {
                double fx = d[0];
                double fy = d[1];
                double m = c.x * fy - c.y * fx;
                cols.add(new double[]{fx, fy, m});
            }
        }
        int M = cols.size();
        DMatrixRMaj G = new DMatrixRMaj(3, M);
        for (int j = 0; j < M; ++j) {
            double[] w = cols.get(j);
            G.set(0, j, w[0]);
            G.set(1, j, w[1]);
            G.set(2, j, w[2]);
        }
        return G;
    }

    // Rank test using SVD or QR can be implemented via EJML.
    public static boolean isFullRank3(DMatrixRMaj G) {
        // Placeholder: use SingularOps_DDRM.rank() in a real implementation.
        return true;
    }

    public static void main(String[] args) {
        List<Contact2D> contacts = new ArrayList<>();
        contacts.add(new Contact2D(1.0, 0.0, 1.0, 0.0));
        contacts.add(new Contact2D(-0.5, Math.sqrt(3.0) / 2.0, -0.5,
                                   Math.sqrt(3.0) / 2.0));
        contacts.add(new Contact2D(-0.5, -Math.sqrt(3.0) / 2.0, -0.5,
                                   -Math.sqrt(3.0) / 2.0));

        double mu = 0.8;
        DMatrixRMaj G = buildWrenchMatrix2D(contacts, mu);
        boolean fullRank = isFullRank3(G);
        System.out.println("Rank test (placeholder): " + fullRank);

        // LP step omitted; use a Java LP solver to complete the test.
    }
}
      
