import org.ejml.simple.SimpleMatrix;

public class RigidMotion {
    public static SimpleMatrix Rz(double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        double[][] data = {
            { c, -s, 0 },
            { s,  c, 0 },
            { 0,  0, 1 }
        };
        return new SimpleMatrix(data);
    }

    public static SimpleMatrix rigidApply(SimpleMatrix R, SimpleMatrix p, SimpleMatrix x) {
        return R.mult(x).plus(p);
    }

    public static SimpleMatrix[] rigidCompose(SimpleMatrix R_cb, SimpleMatrix p_cb,
                                              SimpleMatrix R_ba, SimpleMatrix p_ba) {
        SimpleMatrix R_ca = R_cb.mult(R_ba);
        SimpleMatrix p_ca = R_cb.mult(p_ba).plus(p_cb);
        return new SimpleMatrix[]{R_ca, p_ca};
    }

    public static void main(String[] args) {
        double theta = Math.PI/4.0;
        SimpleMatrix R = Rz(theta);
        SimpleMatrix p = new SimpleMatrix(3,1,true, new double[]{0.5, -0.2, 0.1});
        SimpleMatrix xA = new SimpleMatrix(3,1,true, new double[]{1,0,0});

        SimpleMatrix xB = rigidApply(R, p, xA);
        System.out.println("xB = " + xB);
    }
}
