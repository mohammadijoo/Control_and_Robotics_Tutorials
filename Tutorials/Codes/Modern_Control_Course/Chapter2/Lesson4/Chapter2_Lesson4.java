import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;

public class SimilarityDemo {

    public static SimpleMatrix similarityTransform(SimpleMatrix A, SimpleMatrix T) {
        // A_tilde = T^{-1} A T
        // EJML: use solve to avoid explicit inverse
        SimpleMatrix AT = A.mult(T);
        return T.solve(AT);
    }

    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][]{
            {2.0, 1.0},
            {0.0, 3.0}
        });

        SimpleMatrix T = new SimpleMatrix(new double[][]{
            {1.0, 1.0},
            {0.0, 1.0}
        });

        SimpleMatrix A_tilde = similarityTransform(A, T);

        System.out.println("A =\n" + A);
        System.out.println("A_tilde =\n" + A_tilde);

        double trA = A.trace();
        double trAt = A_tilde.trace();

        System.out.println("trace(A) = " + trA + ", trace(A_tilde) = " + trAt);

        // Eigenvalue check
        SimpleEVD<SimpleMatrix> evA = A.eig();
        SimpleEVD<SimpleMatrix> evAt = A_tilde.eig();

        System.out.println("eig(A):");
        for (int i = 0; i < evA.getNumberOfEigenvalues(); i++) {
            System.out.println(evA.getEigenvalue(i));
        }

        System.out.println("eig(A_tilde):");
        for (int i = 0; i < evAt.getNumberOfEigenvalues(); i++) {
            System.out.println(evAt.getEigenvalue(i));
        }
    }
}
      
