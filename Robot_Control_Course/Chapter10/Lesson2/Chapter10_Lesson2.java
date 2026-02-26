
import org.ojalgo.matrix.Primitive64Matrix;
import org.ojalgo.optimisation.QuadraticSolver;
import org.ojalgo.optimisation.Optimisation;

public class MPCJava {
    public static void main(String[] args) {
        int nx = 4, nu = 2, ny = 2, N = 10;

        // Example A, B, C (you would build them as in Python/C++)
        double[][] Adata = new double[nx][nx];
        double[][] Bdata = new double[nx][nu];
        double[][] Cdata = new double[ny][nx];
        // ... fill Adata, Bdata, Cdata ...

        Primitive64Matrix A = Primitive64Matrix.FACTORY.rows(Adata);
        Primitive64Matrix B = Primitive64Matrix.FACTORY.rows(Bdata);
        Primitive64Matrix C = Primitive64Matrix.FACTORY.rows(Cdata);

        // Build Sx, Su, C_blk, Q_blk, R_blk, etc. (not shown for brevity)
        // Suppose we already have H and f as Primitive64Matrix:
        Primitive64Matrix H = /* 2(T' Q T + R_blk) */;
        Primitive64Matrix f = /* 2 T' Q b */;

        QuadraticSolver.Builder builder = QuadraticSolver.getBuilder();
        builder.quadratic(H);
        builder.linear(f);
        // No constraints set => unconstrained QP

        Optimisation.Result result = builder.build().solve();
        Primitive64Matrix Ustar = Primitive64Matrix.FACTORY.columns(result);

        // Extract first control move
        double[] u0 = new double[nu];
        for (int i = 0; i < nu; ++i) {
            u0[i] = Ustar.get(i, 0);
        }
        System.out.println("u0* = ");
        for (int i = 0; i < nu; ++i) {
            System.out.print(u0[i] + " ");
        }
    }
}
