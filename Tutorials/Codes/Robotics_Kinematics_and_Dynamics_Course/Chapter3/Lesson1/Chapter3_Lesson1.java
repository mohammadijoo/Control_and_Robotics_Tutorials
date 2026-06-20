public final class EulerZYX {

    public static double[][] eulerToR(double phi, double theta, double psi) {
        double cphi = Math.cos(phi), sphi = Math.sin(phi);
        double cth  = Math.cos(theta), sth = Math.sin(theta);
        double cpsi = Math.cos(psi), spsi = Math.sin(psi);

        double[][] R = new double[3][3];
        R[0][0] = cpsi * cth;
        R[0][1] = cpsi * sth * sphi - spsi * cphi;
        R[0][2] = cpsi * sth * cphi + spsi * sphi;

        R[1][0] = spsi * cth;
        R[1][1] = spsi * sth * sphi + cpsi * cphi;
        R[1][2] = spsi * sth * cphi - cpsi * sphi;

        R[2][0] = -sth;
        R[2][1] = cth * sphi;
        R[2][2] = cth * cphi;

        return R;
    }

    public static double[] eulerToOmega(double phi, double theta,
                                        double phiDot, double thetaDot, double psiDot) {
        double cphi = Math.cos(phi), sphi = Math.sin(phi);
        double cth  = Math.cos(theta), sth = Math.sin(theta);

        double[][] T = new double[3][3];
        T[0][0] = 1.0; T[0][1] = 0.0;   T[0][2] = -sth;
        T[1][0] = 0.0; T[1][1] = cphi;  T[1][2] = cth * sphi;
        T[2][0] = 0.0; T[2][1] = -sphi; T[2][2] = cth * cphi;

        double[] qdot = new double[] {phiDot, thetaDot, psiDot};
        double[] omega = new double[3];
        for (int i = 0; i < 3; ++i) {
            omega[i] = 0.0;
            for (int j = 0; j < 3; ++j) {
                omega[i] += T[i][j] * qdot[j];
            }
        }
        return omega;
    }
}
      
