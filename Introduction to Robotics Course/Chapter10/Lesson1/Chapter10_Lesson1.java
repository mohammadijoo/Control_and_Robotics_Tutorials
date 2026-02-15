import org.ejml.simple.SimpleMatrix;

public class LQRStep {
    public static void main(String[] args) {
        SimpleMatrix A = new SimpleMatrix(new double[][]{
            {1.0, 0.01},
            {0.0, 1.0}
        });
        SimpleMatrix B = new SimpleMatrix(new double[][]{
            {0.0},
            {0.01}
        });
        SimpleMatrix K = new SimpleMatrix(new double[][]{
            {2.0, 0.5}
        });

        SimpleMatrix x = new SimpleMatrix(new double[][]{
            {0.1},
            {0.0}
        });

        for(int k=0;k<1000;k++){
            SimpleMatrix u = K.mult(x).scale(-1.0);
            x = A.mult(x).plus(B.mult(u));
        }
        System.out.println(x);
    }
}
