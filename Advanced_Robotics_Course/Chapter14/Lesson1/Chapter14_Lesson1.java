import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Rendezvous {

    public static void centralizedStep(double[] p, double[] p0,
                                       double kGain, double dt) {
        int N = p.length;
        double sum = 0.0;
        for (int i = 0; i < N; ++i) {
            sum += p0[i];
        }
        double pAvg0 = sum / N;

        for (int i = 0; i < N; ++i) {
            double u = -kGain * (p[i] - pAvg0);
            p[i] += dt * u;
        }
    }

    public static void decentralizedStep(double[] p,
                                         List<List<Integer>> neighbors,
                                         double kGain, double dt) {
        int N = p.length;
        double[] u = new double[N];

        for (int i = 0; i < N; ++i) {
            double s = 0.0;
            for (int j : neighbors.get(i)) {
                s += (p[i] - p[j]);
            }
            u[i] = -kGain * s;
        }

        for (int i = 0; i < N; ++i) {
            p[i] += dt * u[i];
        }
    }

    public static void main(String[] args) {
        int N = 4;
        double dt = 0.05;
        double kGain = 1.0;
        double[] p0 = new double[] { -1.0, 0.0, 2.0, 4.0 };
        double[] pCentral = Arrays.copyOf(p0, p0.length);
        double[] pDecent = Arrays.copyOf(p0, p0.length);

        // ring neighbor lists
        List<List<Integer>> neighbors = new ArrayList<>();
        for (int i = 0; i < N; ++i) {
            List<Integer> nbr = new ArrayList<>();
            nbr.add((i - 1 + N) % N);
            nbr.add((i + 1) % N);
            neighbors.add(nbr);
        }

        int steps = 200;
        for (int k = 0; k < steps; ++k) {
            centralizedStep(pCentral, p0, kGain, dt);
            decentralizedStep(pDecent, neighbors, kGain, dt);
        }

        System.out.println("Centralized final: " + Arrays.toString(pCentral));
        System.out.println("Decentralized final: " + Arrays.toString(pDecent));
    }
}
      
