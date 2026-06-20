public class FourBarConstraints {

    private final double a, b, c, d;

    public FourBarConstraints(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    // q = [th2, th3, th4]
    public double[] phi(double[] q) {
        double th2 = q[0];
        double th3 = q[1];
        double th4 = q[2];

        double phi1 = a * Math.cos(th2) + b * Math.cos(th3) - d - c * Math.cos(th4);
        double phi2 = a * Math.sin(th2) + b * Math.sin(th3) - c * Math.sin(th4);
        return new double[]{phi1, phi2};
    }

    // Jc(q) in row-major order, size 2x3
    public double[][] Jc(double[] q) {
        double th2 = q[0];
        double th3 = q[1];
        double th4 = q[2];

        double[][] J = new double[2][3];
        J[0][0] = -a * Math.sin(th2);
        J[0][1] = -b * Math.sin(th3);
        J[0][2] =  c * Math.sin(th4);

        J[1][0] =  a * Math.cos(th2);
        J[1][1] =  b * Math.cos(th3);
        J[1][2] = -c * Math.cos(th4);
        return J;
    }

    public static void main(String[] args) {
        FourBarConstraints fb = new FourBarConstraints(0.2, 0.4, 0.3, 0.5);
        double[] q = {Math.toRadians(10.0), Math.toRadians(60.0), Math.toRadians(30.0)};
        double[] phi = fb.phi(q);
        System.out.printf("phi1 = %.6f, phi2 = %.6f%n", phi[0], phi[1]);

        double[][] J = fb.Jc(q);
        System.out.printf("Jc = [%f %f %f; %f %f %f]%n",
                J[0][0], J[0][1], J[0][2],
                J[1][0], J[1][1], J[1][2]);
    }
}
      
