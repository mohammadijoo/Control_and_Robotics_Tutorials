public class Planar2RJacobian {

    public static double[][] jacobian(double l1, double l2,
                                      double q1, double q2) {
        double[][] J = new double[3][2];

        double c1 = Math.cos(q1);
        double s1 = Math.sin(q1);
        double c12 = Math.cos(q1 + q2);
        double s12 = Math.sin(q1 + q2);

        // omega_z row
        J[0][0] = 1.0;
        J[0][1] = 1.0;

        // v_x row
        J[1][0] = -l1 * s1 - l2 * s12;
        J[1][1] = -l2 * s12;

        // v_y row
        J[2][0] =  l1 * c1 + l2 * c12;
        J[2][1] =  l2 * c12;

        return J;
    }

    public static void main(String[] args) {
        double[][] J = jacobian(1.0, 0.8, 0.5, -0.3);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 2; ++j) {
                System.out.print(J[i][j] + " ");
            }
            System.out.println();
        }
    }
}
      
