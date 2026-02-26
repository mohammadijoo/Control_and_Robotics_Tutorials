public class RigidBodyInertia {
    private double m;
    private double[] com;   // length 3
    private double[][] Icom; // 3x3

    public RigidBodyInertia(double m, double[] com, double[][] Icom) {
        this.m = m;
        this.com = com.clone();
        this.Icom = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            System.arraycopy(Icom[i], 0, this.Icom[i], 0, 3);
        }
    }

    public double[][] inertiaAboutPoint(double[] p) {
        // c_rel = com - p
        double[] c = new double[3];
        for (int i = 0; i < 3; ++i) {
            c[i] = com[i] - p[i];
        }
        double c2 = c[0]*c[0] + c[1]*c[1] + c[2]*c[2];

        double[][] I = new double[3][3];
        // I = I_com + m * (c2 I - c c^T)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double delta = (i == j) ? 1.0 : 0.0;
                I[i][j] = Icom[i][j] + m * (c2 * delta - c[i] * c[j]);
            }
        }
        return I;
    }

    public double[] parameterVectorAboutOrigin() {
        double[] origin = new double[]{0.0, 0.0, 0.0};
        double[][] I = inertiaAboutPoint(origin);
        double Ixx = I[0][0];
        double Iyy = I[1][1];
        double Izz = I[2][2];
        double Ixy = -I[0][1];
        double Ixz = -I[0][2];
        double Iyz = -I[1][2];

        double mcx = m * com[0];
        double mcy = m * com[1];
        double mcz = m * com[2];

        return new double[]{
            m, mcx, mcy, mcz,
            Ixx, Iyy, Izz,
            Ixy, Ixz, Iyz
        };
    }
}
      
