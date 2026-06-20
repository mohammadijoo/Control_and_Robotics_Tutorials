import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

public class OutputInvisibleSubspaceDemo {
    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][]{
                {0, 1, 0},
                {0, 0, 0},
                {0, 0, -1}
        });
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {0}, {1}, {0}
        });
        SimpleMatrix C = new SimpleMatrix(new double[][]{
                {1, 0, 0}
        });

        int n = A.numRows();

        // Build O = [C; C A; ...; C A^(n-1)]
        SimpleMatrix O = new SimpleMatrix(n * C.numRows(), n);
        SimpleMatrix Ak = SimpleMatrix.identity(n);
        for (int k = 0; k < n; k++) {
            SimpleMatrix rowBlock = C.mult(Ak);
            for (int j = 0; j < n; j++) {
                O.set(k, j, rowBlock.get(0, j));
            }
            Ak = Ak.mult(A);
        }

        // SVD for rank and nullspace
        SimpleSVD<SimpleMatrix> svd = O.svd();
        double[] s = svd.getSingularValues();
        double tol = 1e-10;

        int rank = 0;
        for (double value : s) if (value > tol) rank++;

        System.out.println("O =\n" + O);
        System.out.println("rank(O) = " + rank);

        // Nullspace basis from V columns corresponding to small singular values
        SimpleMatrix V = svd.getV();
        int nullDim = n - rank;
        SimpleMatrix NyBasis = V.extractMatrix(0, n, rank, rank + nullDim);

        System.out.println("Ny basis (columns span ker(O)):\n" + NyBasis);
    }
}
      
