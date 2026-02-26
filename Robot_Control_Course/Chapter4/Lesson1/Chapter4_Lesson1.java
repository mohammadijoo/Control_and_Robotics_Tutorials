
public class Planar2DOF {

    public double l1 = 1.0;
    public double l2 = 1.0;

    public double[] fk(double[] q) {
        double q1 = q[0];
        double q2 = q[1];
        double x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        double y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        return new double[]{x, y};
    }

    public double[][] jacobian(double[] q) {
        double q1 = q[0];
        double q2 = q[1];
        double[][] J = new double[2][2];
        J[0][0] = -l1 * Math.sin(q1) - l2 * Math.sin(q1 + q2);
        J[0][1] = -l2 * Math.sin(q1 + q2);
        J[1][0] =  l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
        J[1][1] =  l2 * Math.cos(q1 + q2);
        return J;
    }
}

// Skeleton controller using EJML SimpleMatrix
// (Assume: import org.ejml.simple.SimpleMatrix;)

public class Controllers {

    public static SimpleMatrix inverseDynamics(SimpleMatrix q,
                                               SimpleMatrix qd,
                                               SimpleMatrix qddDes) {
        // Placeholder: in a real implementation, use M(q), C(q, qd), g(q)
        return qddDes;
    }

    public static SimpleMatrix jointSpacePD(SimpleMatrix q,
                                            SimpleMatrix qd,
                                            SimpleMatrix qDes,
                                            SimpleMatrix qdDes,
                                            SimpleMatrix Kp,
                                            SimpleMatrix Kd) {
        SimpleMatrix eQ  = qDes.minus(q);
        SimpleMatrix eQd = qdDes.minus(qd);
        SimpleMatrix qddDes = Kp.mult(eQ).plus(Kd.mult(eQd));
        return inverseDynamics(q, qd, qddDes);
    }

    public static SimpleMatrix taskSpacePD(Planar2DOF arm,
                                           SimpleMatrix q,
                                           SimpleMatrix qd,
                                           SimpleMatrix xDes,
                                           SimpleMatrix xdDes,
                                           SimpleMatrix Kx,
                                           SimpleMatrix Dx) {
        double[] qArr = new double[]{q.get(0), q.get(1)};
        double[] xArr = arm.fk(qArr);
        double[][] JArr = arm.jacobian(qArr);

        SimpleMatrix x = new SimpleMatrix(2, 1, true, xArr);
        SimpleMatrix J = new SimpleMatrix(2, 2, true,
            JArr[0][0], JArr[0][1],
            JArr[1][0], JArr[1][1]);

        SimpleMatrix xdot = J.mult(qd);

        SimpleMatrix eX  = xDes.minus(x);
        SimpleMatrix eXd = xdDes.minus(xdot);

        SimpleMatrix xddDes = Kx.mult(eX).plus(Dx.mult(eXd));

        // Pseudoinverse for 2x2 (non-singular assumption)
        SimpleMatrix Jinv = J.invert();
        SimpleMatrix qddDes = Jinv.mult(xddDes);

        return inverseDynamics(q, qd, qddDes);
    }
}
