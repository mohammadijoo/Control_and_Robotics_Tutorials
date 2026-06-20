public class RollingDiskConstraints {

    private double R;

    public RollingDiskConstraints(double radius) {
        this.R = radius;
    }

    // q = [x, y, theta, phi]
    // qdot = [xdot, ydot, thetadot, phidot]
    public double[] constraintValues(double[] q, double[] qdot) {
        double theta = q[2];
        double xdot = qdot[0];
        double ydot = qdot[1];
        double phidot = qdot[3];

        double c1 = xdot * Math.cos(theta) + ydot * Math.sin(theta) - R * phidot;
        double c2 = xdot * Math.sin(theta) - ydot * Math.cos(theta);

        return new double[]{c1, c2};
    }

    public boolean isAdmissible(double[] q, double[] qdot, double tol) {
        double[] c = constraintValues(q, qdot);
        return Math.abs(c[0]) < tol && Math.abs(c[1]) < tol;
    }

    public static void main(String[] args) {
        RollingDiskConstraints model = new RollingDiskConstraints(0.1);
        double[] q = new double[]{0.0, 0.0, 0.0, 0.0};
        double[] qdot = new double[]{0.2, 0.0, 0.0, 0.2 / 0.1}; // forward rolling

        boolean ok = model.isAdmissible(q, qdot, 1e-6);
        System.out.println("Is admissible velocity? " + ok);
    }
}
      
