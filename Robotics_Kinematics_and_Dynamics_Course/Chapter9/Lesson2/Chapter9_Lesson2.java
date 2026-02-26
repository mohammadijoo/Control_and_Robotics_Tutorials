public class Planar2RVirtualWork {

    public static double[][] jacobian2R(double q1, double q2, double l1, double l2) {
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

    public static double[] jointTorquesFromForce(double q1, double q2,
                                                 double l1, double l2,
                                                 double fx, double fy) {
        double[][] J = jacobian2R(q1, q2, l1, l2);
        double[] tau = new double[2];
        tau[0] = J[0][0]*fx + J[1][0]*fy;
        tau[1] = J[0][1]*fx + J[1][1]*fy;
        return tau;
    }

    public static void main(String[] args) {
        double q1 = Math.PI/4.0;
        double q2 = Math.PI/6.0;
        double l1 = 1.0, l2 = 0.8;
        double fx = 10.0, fy = 0.0;

        double[] tau = jointTorquesFromForce(q1, q2, l1, l2, fx, fy);
        System.out.println("tau1 = " + tau[0] + ", tau2 = " + tau[1]);
    }
}
      
