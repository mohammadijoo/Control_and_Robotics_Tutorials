import org.ejml.simple.SimpleMatrix;

public class NumericalRoboticsExample {
    public static void main(String[] args) {
        // Jacobian-like matrix
        double[][] dataJ = {
            {0.1, 0.0, 0.0},
            {0.0, 1.0, 0.999},
            {0.0, 1.0, 1.001}
        };
        SimpleMatrix J = new SimpleMatrix(dataJ);
        double condJ = J.conditionP2();
        System.out.println("cond(J) = " + condJ);
        if (condJ > 1e6) {
            System.out.println("Warning: ill-conditioned Jacobian");
        }

        // Simple joint integration: qdot = omega, omega_dot = -k*q
        double k = 10.0;
        double q = 0.1;
        double qdot = 0.0;
        double h = 0.001;
        double T = 1.0;
        double t = 0.0;
        while (t < T) {
            double dqdt = qdot;
            double dqdotdt = -k * q;
            q += h * dqdt;
            qdot += h * dqdotdt;
            t += h;
        }
        System.out.println("q(T) = " + q + ", qdot(T) = " + qdot);
    }
}
      
