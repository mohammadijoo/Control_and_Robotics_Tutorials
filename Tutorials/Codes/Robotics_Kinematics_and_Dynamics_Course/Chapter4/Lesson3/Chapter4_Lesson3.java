public class OpenClosedKinematics {

    public static double[] rot2TimesVec(double theta, double[] v) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new double[] {
            c * v[0] - s * v[1],
            s * v[0] + c * v[1]
        };
    }

    public static double[] fkPlanar2R(double theta1, double theta2,
                                      double L1, double L2) {
        double[] e1 = new double[] {L1, 0.0};
        double[] e2 = new double[] {L2, 0.0};

        double[] p1 = rot2TimesVec(theta1, e1);
        double[] e2rot = rot2TimesVec(theta1 + theta2, e2);

        double[] p2 = new double[2];
        p2[0] = p1[0] + e2rot[0];
        p2[1] = p1[1] + e2rot[1];
        return p2;
    }

    public static double fourbarConstraint(double theta1, double theta3,
                                           double L0, double L1,
                                           double L2, double L3) {
        double xB = L1 * Math.cos(theta1);
        double yB = L1 * Math.sin(theta1);
        double xC = L0 + L3 * Math.cos(theta3);
        double yC =        L3 * Math.sin(theta3);
        double dx = xB - xC;
        double dy = yB - yC;
        return dx*dx + dy*dy - L2*L2;
    }

    public static double solveFourbarTheta3(double theta1,
                                            double L0, double L1,
                                            double L2, double L3,
                                            double theta3Init,
                                            double tol, int maxIter) {
        double theta3 = theta3Init;
        for (int k = 0; k < maxIter; ++k) {
            double Phi = fourbarConstraint(theta1, theta3, L0, L1, L2, L3);
            double h = 1e-6;
            double PhiP = fourbarConstraint(theta1, theta3 + h, L0, L1, L2, L3);
            double PhiM = fourbarConstraint(theta1, theta3 - h, L0, L1, L2, L3);
            double dPhi = (PhiP - PhiM) / (2.0 * h);
            if (Math.abs(dPhi) < 1e-14) break;
            double step = Phi / dPhi;
            theta3 -= step;
            if (Math.abs(step) < tol) break;
        }
        return theta3;
    }

    public static void main(String[] args) {
        double theta1 = 0.5, theta2 = -0.3;
        double L1 = 0.5, L2 = 0.4;
        double[] p = fkPlanar2R(theta1, theta2, L1, L2);
        System.out.println("Open-chain end-effector: (" +
                           p[0] + ", " + p[1] + ")");

        double L0 = 0.8, L3 = 0.4, Lmid = 0.7;
        double theta1In = 0.4;
        double theta3 = solveFourbarTheta3(theta1In, L0, L1, Lmid, L3,
                                           0.2, 1e-10, 50);
        System.out.println("Closed-chain theta3: " + theta3);
        System.out.println("Constraint residual: " +
            fourbarConstraint(theta1In, theta3, L0, L1, Lmid, L3));
    }
}
      
