import org.ejml.simple.SimpleMatrix;

public class ImitationLinearPolicy {
    public static void main(String[] args) {
        int N = 10000;
        int d1 = 33;  // feature dim + 1
        int m  = 7;   // joints
        double lambda = 1e-3;

        // Assume these are loaded from files or constructed from data
        SimpleMatrix PhiTilde = loadMatrix("Phi_tilde.txt", N, d1);
        SimpleMatrix A = loadMatrix("A.txt", N, m);

        SimpleMatrix I = SimpleMatrix.identity(d1);
        SimpleMatrix G = PhiTilde.transpose().mult(PhiTilde)
                            .plus(I.scale(N * lambda));
        SimpleMatrix RHS = PhiTilde.transpose().mult(A);

        SimpleMatrix Wtilde = G.solve(RHS);

        int d = d1 - 1;
        SimpleMatrix W = Wtilde.rows(0, m).cols(0, d);
        SimpleMatrix b = Wtilde.cols(d, d1); // (m x 1)

        // Example evaluation for one state
        SimpleMatrix s = new SimpleMatrix(d, 1);
        // ... fill s with features ...
        SimpleMatrix aCmd = W.mult(s).plus(b);

        System.out.println("Action: " + aCmd.transpose());
    }

    private static SimpleMatrix loadMatrix(String path, int rows, int cols) {
        // Implement file parsing as needed
        SimpleMatrix M = new SimpleMatrix(rows, cols);
        // ...
        return M;
    }
}
      
