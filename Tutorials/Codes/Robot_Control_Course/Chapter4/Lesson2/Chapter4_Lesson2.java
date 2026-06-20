
import org.ejml.simple.SimpleMatrix;

public class OpSpaceController {

    public static SimpleMatrix step(
            SimpleMatrix q,
            SimpleMatrix qd,
            SimpleMatrix x,
            SimpleMatrix xd,
            SimpleMatrix x_d,
            SimpleMatrix xd_d,
            SimpleMatrix xdd_d,
            SimpleMatrix Kp,
            SimpleMatrix Kd,
            SimpleMatrix M,
            SimpleMatrix h,
            SimpleMatrix J,
            SimpleMatrix Jdot) {

        // Errors
        SimpleMatrix e  = x.minus(x_d);
        SimpleMatrix ed = xd.minus(xd_d);

        // Desired task acceleration
        SimpleMatrix xdd_ref = xdd_d
                .minus(Kd.mult(ed))
                .minus(Kp.mult(e));

        // Lambda = (J M^-1 J^T)^-1
        SimpleMatrix Minv = M.invert();
        SimpleMatrix JMJT = J.mult(Minv).mult(J.transpose());
        SimpleMatrix Lambda = JMJT.invert();

        // Task-space mu and p
        SimpleMatrix mu = Lambda.mult(J.mult(Minv).mult(h))
                .minus(Lambda.mult(Jdot.mult(qd)));

        SimpleMatrix p = new SimpleMatrix(mu.numRows(), 1);
        p.zero(); // assume gravity is handled in h

        SimpleMatrix F = Lambda.mult(xdd_ref).plus(mu).plus(p);

        // Joint torques tau = J^T F
        SimpleMatrix tau = J.transpose().mult(F);
        return tau;
    }
}
