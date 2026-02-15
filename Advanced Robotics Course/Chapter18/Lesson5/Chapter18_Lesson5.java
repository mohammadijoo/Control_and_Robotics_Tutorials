import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class BenchmarkStats {

    public static class Summary {
        public final double mean;
        public final double lower;
        public final double upper;

        public Summary(double mean, double lower, double upper) {
            this.mean = mean;
            this.lower = lower;
            this.upper = upper;
        }
    }

    public static Summary meanCI95(List<Double> xs) {
        int n = xs.size();
        if (n < 2) {
            return new Summary(Double.NaN, Double.NaN, Double.NaN);
        }
        double sum = 0.0;
        for (double x : xs) sum += x;
        double mean = sum / n;
        double sumSq = 0.0;
        for (double x : xs) {
            double d = x - mean;
            sumSq += d * d;
        }
        double var = sumSq / (n - 1);
        double se = Math.sqrt(var / n);
        double z = 1.96; // approx 95% normal
        return new Summary(mean, mean - z * se, mean + z * se);
    }

    public static void main(String[] args) throws IOException {
        String path = "planning_benchmark.csv";
        List<Double> solveTimesPlannerA = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String header = br.readLine(); // skip header
            String line;
            while ((line = br.readLine()) != null) {
                String[] tokens = line.split(",");
                String planner = tokens[0];
                boolean success = Integer.parseInt(tokens[3]) == 1;
                double solveTime = Double.parseDouble(tokens[5]);
                if (planner.equals("RRTstar") && success) {
                    solveTimesPlannerA.add(solveTime);
                }
            }
        }

        Summary s = meanCI95(solveTimesPlannerA);
        System.out.println("RRT*: mean solve time = " + s.mean
                + " [" + s.lower + ", " + s.upper + "]");
    }
}
      
