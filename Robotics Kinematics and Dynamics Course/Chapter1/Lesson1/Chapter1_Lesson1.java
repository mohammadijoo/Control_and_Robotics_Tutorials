// Using EJML (http://ejml.org) style
import org.ejml.simple.SimpleMatrix;

public class LinearAlgebraDemo {
    public static void main(String[] args) {
        double[][] dataA = {
            {1.0, 1.0, 0.0},
            {0.0, 1.0, 1.0}
        };
        SimpleMatrix A = new SimpleMatrix(dataA);

        System.out.println("A =");
        A.print();

        // SVD for rank and null space
        var svd = A.svd();
        SimpleMatrix W = svd.getW(); // singular values on diagonal
        SimpleMatrix V = svd.getV();

        double tol = 1e-10;
        int rank = 0;
        int n = A.numCols();
        for (int i = 0; i < n; ++i) {
            double sigma = W.get(i, i);
            if (sigma > tol) {
                rank++;
            }
        }
        System.out.println("rank(A) = " + rank);

        System.out.println("Approximate null-space basis:");
        for (int i = 0; i < n; ++i) {
            double sigma = W.get(i, i);
            if (sigma <= tol) {
                System.out.println("basis vector " + i + ":");
                V.extractVector(false, i).print();
            }
        }
    }
}
      
