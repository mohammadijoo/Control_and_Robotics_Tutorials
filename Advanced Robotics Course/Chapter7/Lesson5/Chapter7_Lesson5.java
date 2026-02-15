import java.util.*;
import java.lang.Math;

class Contact {
    double[] p; // length 3
    double[] n; // length 3 (unit)
    double mu;
}

public class GraspScoring {

    static double[] normalize(double[] v) {
        double norm = 0.0;
        for (double x : v) norm += x * x;
        norm = Math.sqrt(norm);
        double[] out = new double[v.length];
        for (int i = 0; i < v.length; ++i) out[i] = v[i] / norm;
        return out;
    }

    static double[] cross(double[] a, double[] b) {
        return new double[] {
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        };
    }

    static double dot(double[] a, double[] b) {
        double s = 0.0;
        for (int i = 0; i < a.length; ++i) s += a[i] * b[i];
        return s;
    }

    static List<double[]> discretizeFrictionCone(double[] n, double mu, int k) {
        double[] nn = normalize(n);
        double[] tmp = new double[] {1.0, 0.0, 0.0};
        if (Math.abs(dot(tmp, nn)) > 0.9) {
            tmp = new double[] {0.0, 1.0, 0.0};
        }
        double[] t1 = normalize(cross(nn, tmp));
        double[] t2 = cross(nn, t1);

        List<double[]> dirs = new ArrayList<>(k);
        for (int i = 0; i < k; ++i) {
            double theta = 2.0 * Math.PI * ((double)i / (double)k);
            double[] t = new double[] {
                Math.cos(theta)*t1[0] + Math.sin(theta)*t2[0],
                Math.cos(theta)*t1[1] + Math.sin(theta)*t2[1],
                Math.cos(theta)*t1[2] + Math.sin(theta)*t2[2]
            };
            double[] f = new double[] {
                nn[0] + mu * t[0],
                nn[1] + mu * t[1],
                nn[2] + mu * t[2]
            };
            dirs.add(normalize(f));
        }
        return dirs;
    }

    static double[][] buildWrenchSet(List<Contact> contacts, int k, double wrenchScale) {
        int M = contacts.size() * k;
        double[][] W = new double[M][6];
        int row = 0;
        for (Contact c : contacts) {
            List<double[]> dirs = discretizeFrictionCone(c.n, c.mu, k);
            for (double[] fdir : dirs) {
                double[] f = new double[] {
                    wrenchScale * fdir[0],
                    wrenchScale * fdir[1],
                    wrenchScale * fdir[2]
                };
                double[] m = cross(c.p, f);
                for (int j = 0; j < 3; ++j) W[row][j] = f[j];
                for (int j = 0; j < 3; ++j) W[row][3 + j] = m[j];
                row++;
            }
        }
        return W;
    }

    static double[][] sampleUnitDirections(int dim, int numSamples, long seed) {
        Random rng = new Random(seed);
        double[][] U = new double[numSamples][dim];
        for (int i = 0; i < numSamples; ++i) {
            double norm = 0.0;
            for (int j = 0; j < dim; ++j) {
                double x = rng.nextGaussian();
                U[i][j] = x;
                norm += x * x;
            }
            norm = Math.sqrt(norm);
            for (int j = 0; j < dim; ++j) {
                U[i][j] /= norm;
            }
        }
        return U;
    }

    static double epsilonQuality(double[][] W, int numDirections) {
        if (W.length == 0) return 0.0;
        double[][] U = sampleUnitDirections(6, numDirections, 42L);
        double eps = Double.POSITIVE_INFINITY;
        for (int i = 0; i < U.length; ++i) {
            double maxProj = -Double.POSITIVE_INFINITY;
            for (int r = 0; r < W.length; ++r) {
                double s = 0.0;
                for (int j = 0; j < 6; ++j) {
                    s += U[i][j] * W[r][j];
                }
                if (s > maxProj) maxProj = s;
            }
            if (maxProj < eps) eps = maxProj;
        }
        return eps;
    }

    public static void main(String[] args) {
        List<Contact> contacts = new ArrayList<>();

        Contact c1 = new Contact();
        c1.p = new double[] {0.05, 0.0, 0.0};
        c1.n = normalize(new double[] {-1.0, 0.0, 0.0});
        c1.mu = 0.7;
        contacts.add(c1);

        Contact c2 = new Contact();
        c2.p = new double[] {-0.05, 0.04, 0.0};
        c2.n = normalize(new double[] {1.0, -1.0, 0.0});
        c2.mu = 0.7;
        contacts.add(c2);

        Contact c3 = new Contact();
        c3.p = new double[] {-0.05, -0.04, 0.0};
        c3.n = normalize(new double[] {1.0, 1.0, 0.0});
        c3.mu = 0.7;
        contacts.add(c3);

        double[][] W = buildWrenchSet(contacts, 8, 1.0);
        double q = epsilonQuality(W, 512);
        System.out.println("Approx epsilon quality: " + q);
    }
}
      
