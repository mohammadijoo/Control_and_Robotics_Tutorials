public class TwoRCoriolis {

    // Compute C(q, qDot) for 2R elbow manipulator
    // q[0] = q1, q[1] = q2
    public static double[][] coriolis(double[] q, double[] qDot,
                                      double a1, double a2, double a3) {
        // a1, a3 unused in Coriolis but included for interface symmetry
        double q2  = q[1];
        double dq1 = qDot[0];
        double dq2 = qDot[1];

        double s2  = Math.sin(q2);
        double h   = -a2 * s2;

        double[][] C = new double[2][2];
        C[0][0] = h * dq2;
        C[0][1] = h * (dq1 + dq2);
        C[1][0] = -h * dq1;
        C[1][1] = 0.0;
        return C;
    }

    // Compute vector C(q, qDot) qDot
    public static double[] coriolisTorque(double[] q, double[] qDot,
                                          double a1, double a2, double a3) {
        double[][] C = coriolis(q, qDot, a1, a2, a3);
        double[] tauC = new double[2];
        tauC[0] = C[0][0] * qDot[0] + C[0][1] * qDot[1];
        tauC[1] = C[1][0] * qDot[0] + C[1][1] * qDot[1];
        return tauC;
    }
}
      
