import java.util.List;
import java.util.ArrayList;

public class CIUtils {

    public static class Summary {
        public int N;
        public double pHat;
        public double ciLow;
        public double ciHigh;
    }

    public static Summary summarizeSuccess(List<Boolean> successes, double alpha) {
        int N = successes.size();
        int count = 0;
        for (boolean s : successes) {
            if (s) {
                count++;
            }
        }
        double pHat = (double) count / (double) N;
        double z = 1.96; // for ~95% CI
        double se = Math.sqrt(pHat * (1.0 - pHat) / (double) N);

        Summary s = new Summary();
        s.N = N;
        s.pHat = pHat;
        s.ciLow = pHat - z * se;
        s.ciHigh = pHat + z * se;
        return s;
    }

    public static void main(String[] args) {
        // Example: successes for a planner in 10 trials
        List<Boolean> successes = new ArrayList<>();
        boolean[] arr = {true, true, false, true, true, true, false, true, true, true};
        for (boolean b : arr) {
            successes.add(b);
        }

        Summary s = summarizeSuccess(successes, 0.05);
        System.out.println("N      = " + s.N);
        System.out.println("p_hat  = " + s.pHat);
        System.out.println("CI     = [" + s.ciLow + ", " + s.ciHigh + "]");
    }
}
      
