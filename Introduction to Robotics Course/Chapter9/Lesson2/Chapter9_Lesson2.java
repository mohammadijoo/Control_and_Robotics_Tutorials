import org.ejml.simple.SimpleMatrix;

public class FrameTransform {
    public static void main(String[] args) {
        SimpleMatrix R_WB = new SimpleMatrix(new double[][]{
            {0, -1, 0},
            {1,  0, 0},
            {0,  0, 1}
        });

        SimpleMatrix t_WB = new SimpleMatrix(3,1,true, new double[]{1.0, 2.0, 0.5});
        SimpleMatrix p_B  = new SimpleMatrix(3,1,true, new double[]{0.3, 0.0, 0.2});

        SimpleMatrix p_W = R_WB.mult(p_B).plus(t_WB);
        System.out.println("p_W = " + p_W);

        SimpleMatrix p_B_rec = R_WB.transpose().mult(p_W.minus(t_WB));
        System.out.println("p_B recovered = " + p_B_rec);
    }
}
