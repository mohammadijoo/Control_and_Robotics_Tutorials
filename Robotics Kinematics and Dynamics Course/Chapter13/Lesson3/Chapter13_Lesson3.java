import org.ejml.simple.SimpleMatrix;

public class BaseParameterDemo {
    public static void main(String[] args) {
        int N = 50;  // samples
        int p = 4;   // full parameters

        // Build a random regressor W (replace with real robot regressor)
        SimpleMatrix W = SimpleMatrix.random_DDRM(N, p, -1.0, 1.0, null);

        // Compute SVD: W = U * S * V^T
        var svd = W.svd();
        SimpleMatrix V = svd.getV();
        double[] s = svd.getW().extractDiag().getDDRM().getData();

        // Determine numerical rank
        double tol = 1e-8;
        int r = 0;
        for (double value : s) {
            if (value > tol) {
                r++;
            }
        }
        System.out.println("Rank r = " + r);

        // Extract V1 (first r columns) and V2 (remaining columns)
        SimpleMatrix V1 = V.extractMatrix(0, SimpleMatrix.END, 0, r);
        SimpleMatrix V2 = V.extractMatrix(0, SimpleMatrix.END, r, p);

        // Example full parameter vector
        SimpleMatrix piFull = new SimpleMatrix(p, 1, true,
                                               0.1, 2.0, 0.5, 1.0);
        // Base parameters beta = V1^T * pi
        SimpleMatrix beta = V1.transpose().mult(piFull);

        System.out.println("beta = ");
        beta.print();
        System.out.println("Null-space basis V2 = ");
        V2.print();
    }
}
      
