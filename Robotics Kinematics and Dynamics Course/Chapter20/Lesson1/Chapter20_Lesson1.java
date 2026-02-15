import org.ejml.simple.SimpleMatrix;

public class GeometricRobotModel {

    // Compute kinetic energy T(q, dq) = 0.5 * dq^T M(q) dq
    public static double kineticEnergy(SimpleMatrix Mq, SimpleMatrix dq) {
        SimpleMatrix temp = dq.transpose().mult(Mq).mult(dq);
        return 0.5 * temp.get(0, 0);
    }

    public static void main(String[] args) {
        // Example 2-DOF metric at some configuration q
        double[][] Mdata = {
            {2.0, 0.5},
            {0.5, 1.0}
        };
        double[][] dqdata = {
            {0.3},
            {-0.1}
        };

        SimpleMatrix Mq = new SimpleMatrix(Mdata);
        SimpleMatrix dq = new SimpleMatrix(dqdata);

        double T = kineticEnergy(Mq, dq);
        // T encodes the Riemannian norm of dq
    }
}
      
