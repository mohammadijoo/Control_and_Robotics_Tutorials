import java.util.List;

public class Mason {

    // Compute Delta for scalar loop gains and precomputed products of non-touching loops
    public static double computeDelta(
            List<Double> loops,
            List<List<Integer>> nonTouchingSets) {

        double delta = 1.0;

        for (double L : loops) {
            delta -= L;
        }

        for (List<Integer> set : nonTouchingSets) {
            double prod = 1.0;
            for (int idx : set) {
                prod *= loops.get(idx);
            }
            int m = set.size();
            double sign = Math.pow(-1.0, m);
            delta += sign * prod;
        }
        return delta;
    }

    // Simple feedback example: T = G / (1 + G*H)
    public static double simpleFeedback(double G, double H) {
        double loopGain = -G * H;
        double delta = 1.0 - loopGain; // 1 + G*H
        return G / delta;
    }

    public static void main(String[] args) {
        double G = 10.0;
        double H = 0.5;
        double T = simpleFeedback(G, H);
        System.out.println("Closed-loop gain: " + T);
    }
}
