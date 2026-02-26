public class RootLocusCase1 {

    public static void main(String[] args) {
        double Kc = 0.05;  // proportional gain from Case Study 1

        // Characteristic: s^2 + 10 s + 1000 Kc = 0
        double a = 1.0;
        double b = 10.0;
        double c = 1000.0 * Kc;

        double disc = b * b - 4.0 * a * c;

        // Complex roots
        double realPart = -b / (2.0 * a);
        double imagPart = Math.sqrt(Math.abs(disc)) / (2.0 * a);

        System.out.println("Closed-loop poles for Kc = " + Kc + ":");
        System.out.println("s1 = " + realPart + " + j" + imagPart);
        System.out.println("s2 = " + realPart + " - j" + imagPart);

        // In a Java-based robot simulator, these poles can be used to set
        // integration step sizes or to compare candidate controller designs
        // before deployment to embedded hardware.
    }
}
