import org.ejml.simple.SimpleMatrix;
import org.ejml.data.Complex_F64;
import org.ejml.dense.row.decomposition.eig.EigenOps_DDRM;

public class EigenExample {
    public static void main(String[] args) {
        // 2x2 system matrix (e.g. linearization of a planar joint subsystem)
        double[][] dataA = {
                {0.9, 0.1},
                {0.0, 0.8}
        };

        SimpleMatrix A = new SimpleMatrix(dataA);

        // Compute eigen decomposition using EJML low-level API
        var eig = org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_DDRM
                .decompose(A.getDDRM(), true);

        int N = A.numRows();
        for (int i = 0; i < N; ++i) {
            Complex_F64 lambda = eig.getEigenvalue(i);
            System.out.println("Eigenvalue " + i + ": " + lambda.real + " + " + lambda.imaginary + "i");

            var v = eig.getEigenVector(i);
            if (v == null) {
                System.out.println("No eigenvector (defective eigenvalue)");
            } else {
                System.out.print("Eigenvector " + i + ": [");
                for (int j = 0; j < N; ++j) {
                    System.out.print(v.get(j) + (j + 1 < N ? ", " : ""));
                }
                System.out.println("]");
            }
        }
    }
}
      
