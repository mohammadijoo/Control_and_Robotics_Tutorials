public class Planar2RStatics {

    static double gAcc = 9.81;

    static double potential(double[] q, double l1, double l2,
                            double m1, double m2) {
        double q1 = q[0];
        double q2 = q[1];
        double y1 = (l1 / 2.0) * Math.sin(q1);
        double y2 = l1 * Math.sin(q1) + (l2 / 2.0) * Math.sin(q1 + q2);
        return m1 * gAcc * y1 + m2 * gAcc * y2;
    }

    static double[] gravityTorque(double[] q, double l1, double l2,
                                  double m1, double m2) {
        double eps = 1e-6;
        double[] grad = new double[2];
        for (int i = 0; i < 2; ++i) {
            double[] qp = q.clone();
            double[] qm = q.clone();
            qp[i] += eps;
            qm[i] -= eps;
            double Vp = potential(qp, l1, l2, m1, m2);
            double Vm = potential(qm, l1, l2, m1, m2);
            grad[i] = (Vp - Vm) / (2.0 * eps);
        }
        return grad;
    }

    static double[][] jacobian2R(double[] q, double l1, double l2) {
        double q1 = q[0];
        double q2 = q[1];
        double s1 = Math.sin(q1);
        double c1 = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] J = new double[2][2];
        J[0][0] = -l1 * s1 - l2 * s12;
        J[0][1] = -l2 * s12;
        J[1][0] =  l1 * c1 + l2 * c12;
        J[1][1] =  l2 * c12;
        return J;
    }

    static double[] constrainedStaticTorque(double[] q,
                                            double l1, double l2,
                                            double m1, double m2,
                                            double[] wrench6,
                                            double lambda) {
        double[] gTau = gravityTorque(q, l1, l2, m1, m2);
        double[][] Jxy = jacobian2R(q, l1, l2);

        // J is 6x2, only first two rows nonzero
        double[] tauExt = new double[2];
        tauExt[0] = Jxy[0][0] * wrench6[0] + Jxy[1][0] * wrench6[1];
        tauExt[1] = Jxy[0][1] * wrench6[0] + Jxy[1][1] * wrench6[1];

        double q1 = q[0];
        double q2 = q[1];
        double jphi0 = -l1 * Math.sin(q1) - l2 * Math.sin(q1 + q2);
        double jphi1 = -l2 * Math.sin(q1 + q2);

        double[] tauCon = new double[2];
        tauCon[0] = jphi0 * lambda;
        tauCon[1] = jphi1 * lambda;

        double[] tau = new double[2];
        for (int i = 0; i < 2; ++i) {
            tau[i] = gTau[i] + tauExt[i] + tauCon[i];
        }
        return tau;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 0.7;
        double m1 = 2.0, m2 = 1.0;
        double[] q = {
            Math.toRadians(40.0),
            Math.toRadians(30.0)
        };
        double[] wrench6 = new double[6];
        wrench6[0] = 0.0;   // Fx
        wrench6[1] = -20.0; // Fy
        double lambda = 50.0;

        double[] tau = constrainedStaticTorque(q, l1, l2, m1, m2, wrench6, lambda);
        System.out.println("tau1 = " + tau[0] + ", tau2 = " + tau[1]);
    }
}
      
