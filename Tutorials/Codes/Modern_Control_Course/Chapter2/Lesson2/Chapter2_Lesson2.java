import org.ejml.simple.SimpleMatrix;

public class LinearMapDemo {
    public static void main(String[] args) {
        // A in R^{2x3}
        SimpleMatrix A = new SimpleMatrix(new double[][]{
                {1, 2, 0},
                {0, -1, 3}
        });

        SimpleMatrix x = new SimpleMatrix(new double[][]{
                {1.0},
                {-2.0},
                {0.5}
        });

        SimpleMatrix Tx = A.mult(x);

        System.out.println("A=\n" + A);
        System.out.println("x=\n" + x);
        System.out.println("T(x)=A x=\n" + Tx);

        // Composition with B in R^{2x2}
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {0, 1},
                {-1, 0}
        });

        SimpleMatrix BA = B.mult(A);
        System.out.println("BA=\n" + BA);
        System.out.println("(S o T)(x)=BA x=\n" + BA.mult(x));
    }
}
      
