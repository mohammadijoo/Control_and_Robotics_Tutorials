import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class MassSpringDamper {
    public static void main(String[] args) {
        double[][] Adata = {
            {0.0, 1.0},
            {-2.0, -0.4}  // m = 1, k = 2, c = 0.4
        };
        double[][] Bdata = {
            {0.0},
            {1.0}
        };

        RealMatrix A = new Array2DRowRealMatrix(Adata);
        RealMatrix B = new Array2DRowRealMatrix(Bdata);
        RealVector x = new ArrayRealVector(new double[]{0.0, 0.0});

        double u = 1.0;
        double dt = 0.001;
        int steps = 10000;

        for (int k = 0; k < steps; ++k) {
            RealVector xdot = A.operate(x).add(B.operate(new ArrayRealVector(new double[]{u})));
            x = x.add(xdot.mapMultiply(dt));
        }

        System.out.println("Final state x = " + x);
    }
}
      
