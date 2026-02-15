import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.linear.Array2DRowFieldMatrix;
import org.apache.commons.math3.linear.ArrayFieldVector;
import org.apache.commons.math3.linear.FieldDecompositionSolver;
import org.apache.commons.math3.linear.FieldLUDecomposition;
import org.apache.commons.math3.linear.FieldVector;

public class PartialFractionJava {

    // Example: F(s) = 1 / (s*(s+3))
    public static Complex F(Complex s) {
        return Complex.ONE.divide(s.multiply(s.add(new Complex(3.0, 0.0))));
    }

    public static void main(String[] args) {
        Complex s0 = new Complex(0.5, 0.0);
        Complex s1 = new Complex(1.0, 0.0);

        Complex[][] data = new Complex[][] {
            { Complex.ONE.divide(s0), Complex.ONE.divide(s0.add(new Complex(3.0, 0.0))) },
            { Complex.ONE.divide(s1), Complex.ONE.divide(s1.add(new Complex(3.0, 0.0))) }
        };
        Complex[] rhs = new Complex[] { F(s0), F(s1) };

        Array2DRowFieldMatrix<Complex> M = new Array2DRowFieldMatrix<>(data);
        ArrayFieldVector<Complex> b = new ArrayFieldVector<>(rhs);

        FieldDecompositionSolver<Complex> solver =
            new FieldLUDecomposition<>(M).getSolver();
        FieldVector<Complex> x = solver.solve(b);

        Complex A = x.getEntry(0);
        Complex B = x.getEntry(1);

        System.out.println("A = " + A);
        System.out.println("B = " + B);

        // In control design tools for Java-based robots (e.g., FRC libraries),
        // such rational models can be used to predict time responses.
    }
}
