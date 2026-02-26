public class RealZeroDesign {
    public static void main(String[] args) {
        // Desired dominant pole sd = -2 + j*3.464...
        double sr = -2.0;
        double si =  3.464101615;
        double zc =  8.0;    // zero at s = -8

        // Magnitudes needed for K = |s (s + 2)| / |s + zc|
        double abs_s = Math.hypot(sr, si);
        double abs_s_plus_2 = Math.hypot(sr + 2.0, si);
        double abs_s_plus_zc = Math.hypot(sr + zc, si);

        double K = (abs_s * abs_s_plus_2) / abs_s_plus_zc;
        System.out.println("Designed gain K ≈ " + K);
    }
}
