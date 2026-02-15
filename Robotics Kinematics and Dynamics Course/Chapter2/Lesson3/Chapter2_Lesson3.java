import org.ejml.simple.SimpleMatrix;

public class SE3 {

    public static SimpleMatrix makeTransform(SimpleMatrix R, SimpleMatrix p) {
        // R: 3x3, p: 3x1
        SimpleMatrix T = SimpleMatrix.identity(4);
        T.insertIntoThis(0, 0, R);
        T.insertIntoThis(0, 3, p);
        return T;
    }

    public static SimpleMatrix inverseTransform(SimpleMatrix T) {
        SimpleMatrix R = T.extractMatrix(0, 3, 0, 3);
        SimpleMatrix p = T.extractMatrix(0, 3, 3, 4);
        SimpleMatrix R_t = R.transpose();

        SimpleMatrix Tinv = SimpleMatrix.identity(4);
        Tinv.insertIntoThis(0, 0, R_t);
        Tinv.insertIntoThis(0, 3, R_t.mult(p).scale(-1.0));
        return Tinv;
    }

    public static SimpleMatrix transformPoint(SimpleMatrix T, SimpleMatrix x) {
        // x: 3x1
        SimpleMatrix x_h = new SimpleMatrix(4, 1, true,
                new double[]{x.get(0), x.get(1), x.get(2), 1.0});
        SimpleMatrix y_h = T.mult(x_h);
        return y_h.extractMatrix(0, 3, 0, 1);
    }

    public static void main(String[] args) {
        // Example: identity rotation, translation along x
        SimpleMatrix R = SimpleMatrix.identity(3);
        SimpleMatrix p = new SimpleMatrix(3, 1, true, new double[]{0.5, 0.0, 0.0});

        SimpleMatrix T = makeTransform(R, p);
        SimpleMatrix x = new SimpleMatrix(3, 1, true, new double[]{0.1, 0.0, 0.0});
        SimpleMatrix y = transformPoint(T, x);

        System.out.println("y = ");
        y.print();
    }
}
      
