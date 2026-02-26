import org.ejml.simple.SimpleMatrix;

public class FrameConsistency {

    static SimpleMatrix makeT(SimpleMatrix R, SimpleMatrix p) {
        SimpleMatrix T = SimpleMatrix.identity(4);
        T.insertIntoThis(0,0,R);
        T.insertIntoThis(0,3,p);
        return T;
    }

    static SimpleMatrix invT(SimpleMatrix T) {
        SimpleMatrix R = T.extractMatrix(0,3,0,3);
        SimpleMatrix p = T.extractMatrix(0,3,3,4);
        SimpleMatrix Tinv = SimpleMatrix.identity(4);
        Tinv.insertIntoThis(0,0,R.transpose());
        Tinv.insertIntoThis(0,3,R.transpose().scale(-1).mult(p));
        return Tinv;
    }

    static double frobError(SimpleMatrix E) {
        return E.minus(SimpleMatrix.identity(4)).normF();
    }

    public static void main(String[] args) {
        SimpleMatrix R_WB = SimpleMatrix.identity(3);
        SimpleMatrix p_WB = new SimpleMatrix(3,1,true, new double[]{1,0,0});
        SimpleMatrix T_WB = makeT(R_WB, p_WB);

        SimpleMatrix R_BC = SimpleMatrix.identity(3);
        SimpleMatrix p_BC = new SimpleMatrix(3,1,true, new double[]{0,2,0});
        SimpleMatrix T_BC = makeT(R_BC, p_BC);

        SimpleMatrix T_WC_derived = T_WB.mult(T_BC);
        SimpleMatrix T_WC_meas = T_WC_derived.copy();
        T_WC_meas.set(0,3, T_WC_meas.get(0,3) + 0.05);

        SimpleMatrix E = T_WB.mult(T_BC).mult(invT(T_WC_meas));
        System.out.println("Cycle Frobenius error: " + frobError(E));
    }
}
