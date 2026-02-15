import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class LqrStateFeedback {
    public static void main(String[] args) {
        double[][] Adata = {
            {0.0, 1.0},
            {-2.0, -0.5}
        };
        double[][] Bdata = {
            {0.0},
            {1.0}
        };
        double[][] Kdata = {
            {3.0, 1.2}  // row vector
        };

        RealMatrix A = new Array2DRowRealMatrix(Adata);
        RealMatrix B = new Array2DRowRealMatrix(Bdata);
        RealMatrix K = new Array2DRowRealMatrix(Kdata);

        double[][] xdata = {
            {0.1},
            {0.0}
        };
        RealMatrix x = new Array2DRowRealMatrix(xdata);

        // u = -K x
        RealMatrix u = K.multiply(x).scalarMultiply(-1.0);
        System.out.println("u = " + u.getEntry(0, 0));

        // one-step Euler integration
        RealMatrix xdot = A.multiply(x).add(B.multiply(u));
        double dt = 0.001;
        RealMatrix xNext = x.add(xdot.scalarMultiply(dt));
        System.out.println("xNext = ["
            + xNext.getEntry(0, 0) + ", "
            + xNext.getEntry(1, 0) + "]");
    }
}
