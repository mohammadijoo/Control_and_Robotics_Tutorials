import org.ejml.simple.SimpleMatrix;

public class FrameChange2D {
    public static void main(String[] args) {
        SimpleMatrix R_BA = new SimpleMatrix(new double[][] {
            {0, 1},
            {-1, 0}
        });

        SimpleMatrix v_A = new SimpleMatrix(2, 1, true, new double[] {2.0, 1.0});
        SimpleMatrix v_B = R_BA.mult(v_A);

        System.out.println("v_A = \n" + v_A);
        System.out.println("v_B = \n" + v_B);
        System.out.println("lengths: " +
            v_A.normF() + " " + v_B.normF());
    }
}
