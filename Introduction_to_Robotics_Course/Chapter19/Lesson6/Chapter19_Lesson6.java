import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class RobotStats {
    public static void main(String[] args) throws IOException {
        Map<Integer, List<Double>> data = new HashMap<>();

        try (BufferedReader br = new BufferedReader(new FileReader("robot_logs.csv"))) {
            String line = br.readLine(); // header
            while ((line = br.readLine()) != null) {
                String[] tokens = line.split(",");
                double yRef = Double.parseDouble(tokens[1]);
                double yMeas = Double.parseDouble(tokens[2]);
                int trialId = Integer.parseInt(tokens[4]);
                double e = yRef - yMeas;

                data.computeIfAbsent(trialId, k -> new ArrayList<>()).add(e);
            }
        }

        List<Double> Jvalues = new ArrayList<>();
        for (Map.Entry<Integer, List<Double>> entry : data.entrySet()) {
            int trialId = entry.getKey();
            List<Double> samples = entry.getValue();
            double sumSq = 0.0;
            for (double e : samples) {
                sumSq += e * e;
            }
            double J = Math.sqrt(sumSq / samples.size());
            Jvalues.add(J);
            System.out.println("Trial " + trialId + " RMS error = " + J);
        }

        int N = Jvalues.size();
        double mean = 0.0;
        for (double v : Jvalues) mean += v;
        mean /= N;

        double var = 0.0;
        for (double v : Jvalues) {
            double d = v - mean;
            var += d * d;
        }
        var /= (N - 1);
        double stddev = Math.sqrt(var);

        System.out.println("Mean RMS error = " + mean);
        System.out.println("Std of RMS error = " + stddev);
    }
}
      
