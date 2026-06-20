public class StabilityBoundary {

    // First-column Routh entries for p(s,K) = s^3 + 6 s^2 + 8 s + K
    public static boolean isStable(double K) {
        double r3 = 1.0;
        double r2 = 6.0;
        double r1 = (48.0 - K) / 6.0;
        double r0 = K;
        return (r3 > 0.0) && (r2 > 0.0) && (r1 > 0.0) && (r0 > 0.0);
    }

    public static void main(String[] args) {
        double Kcrit = 48.0;
        System.out.println("Critical gain from Routh: Kcrit = " + Kcrit);

        // Scan around Kcrit to illustrate loss of stability
        for (double K = 30.0; K <= 60.0; K += 5.0) {
            System.out.println("K = " + K + (isStable(K) ? " : stable" : " : unstable"));
        }

        // In a Java-based robotic control framework, K could be tuned online
        // while enforcing this stability constraint.
    }
}
