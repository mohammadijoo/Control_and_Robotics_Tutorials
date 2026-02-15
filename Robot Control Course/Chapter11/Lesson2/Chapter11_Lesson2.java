
import org.ejml.simple.SimpleMatrix;

public class JointObserver2DOF {
    private SimpleMatrix Ad;   // 4x4
    private SimpleMatrix Bd;   // 4x2
    private SimpleMatrix C;    // 2x4
    private SimpleMatrix Ld;   // 4x2
    private SimpleMatrix xHat; // 4x1

    public JointObserver2DOF() {
        Ad = SimpleMatrix.identity(4);
        Ad.set(0, 1, 0.001);
        Ad.set(2, 3, 0.001);
        // ... fill remaining entries ...

        Bd = new SimpleMatrix(4, 2);
        Bd.zero();
        Bd.set(1, 0, 0.005);
        Bd.set(3, 1, 0.005);

        C = new SimpleMatrix(2, 4);
        C.zero();
        C.set(0, 0, 1.0); // q1
        C.set(1, 2, 1.0); // q2

        Ld = new SimpleMatrix(4, 2);
        Ld.set(0, 0, 0.8);
        Ld.set(1, 0, 50.0);
        Ld.set(2, 1, 0.8);
        Ld.set(3, 1, 50.0);

        xHat = new SimpleMatrix(4, 1);
        xHat.zero();
    }

    public void reset(SimpleMatrix x0Hat) {
        xHat = x0Hat.copy();
    }

    public void update(SimpleMatrix yMeas, SimpleMatrix u) {
        // yMeas: 2x1; u: 2x1
        SimpleMatrix yHat = C.mult(xHat);
        SimpleMatrix innovation = yMeas.minus(yHat);
        xHat = Ad.mult(xHat).plus(Bd.mult(u)).plus(Ld.mult(innovation));
    }

    public SimpleMatrix getStateEstimate() {
        return xHat;
    }
}
