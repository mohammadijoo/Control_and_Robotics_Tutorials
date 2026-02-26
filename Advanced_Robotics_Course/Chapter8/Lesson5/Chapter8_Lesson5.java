import org.ejml.simple.SimpleMatrix;
import java.util.ArrayList;
import java.util.List;

class Vec3 {
    public double x, y, z;
    public Vec3(double x, double y, double z) {
        this.x = x; this.y = y; this.z = z;
    }
}

public class GraspQuality {

    public static SimpleMatrix skew(Vec3 v) {
        double[][] data = {
                { 0.0,   -v.z,  v.y },
                { v.z,    0.0, -v.x },
                { -v.y,   v.x,  0.0 }
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix buildGraspMap(List<Vec3> contacts) {
        int m = contacts.size();
        SimpleMatrix G_top = new SimpleMatrix(3, 3 * m);
        SimpleMatrix G_bottom = new SimpleMatrix(3, 3 * m);
        G_top.zero(); G_bottom.zero();

        for (int i = 0; i < m; ++i) {
            int idx = 3 * i;
            // Set identity block
            for (int r = 0; r < 3; ++r) {
                G_top.set(r, idx + r, 1.0);
            }
            SimpleMatrix S = skew(contacts.get(i));
            G_bottom.insertIntoThis(0, idx, S);
        }
        // Stack vertically
        SimpleMatrix G = new SimpleMatrix(6, 3 * m);
        G.insertIntoThis(0, 0, G_top);
        G.insertIntoThis(3, 0, G_bottom);
        return G;
    }

    public static double epsilonSurrogate(SimpleMatrix G) {
        SimpleMatrix[] svd = G.svd();
        SimpleMatrix S = svd[1]; // diagonal singular values
        double minSigma = Double.POSITIVE_INFINITY;
        int n = Math.min(S.numRows(), S.numCols());
        for (int i = 0; i < n; ++i) {
            double sigma = S.get(i, i);
            if (sigma < minSigma) {
                minSigma = sigma;
            }
        }
        return minSigma;
    }

    public static void main(String[] args) {
        List<Vec3> contacts = new ArrayList<>();
        contacts.add(new Vec3(0.0,  0.03, 0.0));
        contacts.add(new Vec3(0.0, -0.03, 0.0));

        SimpleMatrix G = buildGraspMap(contacts);
        double q = epsilonSurrogate(G);
        System.out.println("sigma_min(G) = " + q);
    }
}
      
