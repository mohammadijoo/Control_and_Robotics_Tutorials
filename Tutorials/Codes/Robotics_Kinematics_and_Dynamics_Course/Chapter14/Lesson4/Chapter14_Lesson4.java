public class TorqueSaturation {

    public static double saturateScalar(double u, double uMin, double uMax) {
        if (u > uMax) {
            return uMax;
        } else if (u < uMin) {
            return uMin;
        } else {
            return u;
        }
    }

    public static double[] saturateVector(
            double[] tauCmd,
            double[] tauMin,
            double[] tauMax) {

        int n = tauCmd.length;
        double[] tau = new double[n];
        for (int i = 0; i < n; i++) {
            double u = tauCmd[i];
            double uMin = tauMin[i];
            double uMax = tauMax[i];
            if (u > uMax) {
                tau[i] = uMax;
            } else if (u < uMin) {
                tau[i] = uMin;
            } else {
                tau[i] = u;
            }
        }
        return tau;
    }
}
      
