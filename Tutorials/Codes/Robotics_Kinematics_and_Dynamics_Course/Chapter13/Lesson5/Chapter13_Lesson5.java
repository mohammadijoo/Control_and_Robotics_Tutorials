public final class LinkInertial {
    private final double mass;
    private final double[] com;  // length 3
    private final double[][] Iorigin; // 3x3 inertia about origin

    public LinkInertial(double mass, double[] com, double[][] Iorigin) {
        if (com.length != 3) {
            throw new IllegalArgumentException("com must have length 3");
        }
        if (Iorigin.length != 3 || Iorigin[0].length != 3) {
            throw new IllegalArgumentException("Iorigin must be 3x3");
        }
        this.mass = mass;
        this.com = com.clone();
        this.Iorigin = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            System.arraycopy(Iorigin[i], 0, this.Iorigin[i], 0, 3);
        }
    }

    public double[] inertialParameters10d() {
        double[] pi = new double[10];
        pi[0] = mass;
        pi[1] = mass * com[0];
        pi[2] = mass * com[1];
        pi[3] = mass * com[2];
        pi[4] = Iorigin[0][0];      // Ixx
        pi[5] = Iorigin[1][1];      // Iyy
        pi[6] = Iorigin[2][2];      // Izz
        pi[7] = Iorigin[0][1];      // Ixy
        pi[8] = Iorigin[1][2];      // Iyz
        pi[9] = Iorigin[0][2];      // Ixz
        return pi;
    }
}
      
