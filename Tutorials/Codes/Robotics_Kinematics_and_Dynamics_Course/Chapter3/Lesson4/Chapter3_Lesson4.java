public final class SO3Conversions {

    public static double[][] quatToR(double w, double x, double y, double z) {
        // Normalize
        double n = Math.sqrt(w*w + x*x + y*y + z*z);
        w /= n; x /= n; y /= n; z /= n;

        double[][] R = new double[3][3];
        R[0][0] = 1.0 - 2.0*(y*y + z*z);
        R[0][1] = 2.0*(x*y - w*z);
        R[0][2] = 2.0*(x*z + w*y);

        R[1][0] = 2.0*(x*y + w*z);
        R[1][1] = 1.0 - 2.0*(x*x + z*z);
        R[1][2] = 2.0*(y*z - w*x);

        R[2][0] = 2.0*(x*z - w*y);
        R[2][1] = 2.0*(y*z + w*x);
        R[2][2] = 1.0 - 2.0*(x*x + y*y);
        return R;
    }

    public static double[] RToEulerZYX(double[][] R) {
        double r31 = R[2][0];
        double phi, theta, psi;
        if (Math.abs(r31) < 1.0 - 1e-8) {
            theta = Math.asin(-r31);
            phi   = Math.atan2(R[2][1], R[2][2]);
            psi   = Math.atan2(R[1][0], R[0][0]);
        } else {
            // Gimbal lock
            if (r31 < 0) {
                theta = Math.PI / 2.0;
                phi = 0.0;
                psi = Math.atan2(-R[0][1], R[1][1]);
            } else {
                theta = -Math.PI / 2.0;
                phi = 0.0;
                psi = Math.atan2(R[0][1], R[1][1]);
            }
        }
        return new double[]{phi, theta, psi};
    }
}
      
