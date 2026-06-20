public class Planar2RSingularity {

    public static double detJacobian2R(double theta1, double theta2,
                                       double l1, double l2) {
        return l1 * l2 * Math.sin(theta2);
    }

    public static boolean isKinematicSingular(double theta1, double theta2,
                                              double l1, double l2,
                                              double tol) {
        double detJ = detJacobian2R(theta1, theta2, l1, l2);
        return Math.abs(detJ) < tol;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 1.0;
        double theta1 = 0.0;
        double theta2 = Math.PI; // folded singularity

        double detJ = detJacobian2R(theta1, theta2, l1, l2);
        System.out.println("detJ = " + detJ);
        System.out.println("singular? "
                           + (isKinematicSingular(theta1, theta2, l1, l2, 1e-6)
                              ? "yes" : "no"));
    }
}
      
