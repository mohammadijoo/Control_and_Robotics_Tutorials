// Chapter14_Lesson5.java
// Limit Cycles, Multiple Equilibria, and Basic Bifurcation Notions
// Java implementation: RK4 simulation of Van der Pol + basic bifurcation classifications
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Chapter14_Lesson5 {
    static class State {
        double x, y;
        State(double x, double y) { this.x = x; this.y = y; }
    }

    static State rhsVanDerPol(State s, double mu) {
        double dx = s.y;
        double dy = mu * (1.0 - s.x * s.x) * s.y - s.x;
        return new State(dx, dy);
    }

    static State add(State a, State b, double scale) {
        return new State(a.x + scale * b.x, a.y + scale * b.y);
    }

    static State rk4Step(State s, double h, double mu) {
        State k1 = rhsVanDerPol(s, mu);
        State k2 = rhsVanDerPol(add(s, k1, 0.5 * h), mu);
        State k3 = rhsVanDerPol(add(s, k2, 0.5 * h), mu);
        State k4 = rhsVanDerPol(add(s, k3, h), mu);
        return new State(
            s.x + (h / 6.0) * (k1.x + 2*k2.x + 2*k3.x + k4.x),
            s.y + (h / 6.0) * (k1.y + 2*k2.y + 2*k3.y + k4.y)
        );
    }

    static String pitchforkStability(double r, double xeq) {
        double lambda = r - 3.0 * xeq * xeq;
        if (lambda < 0) return "stable";
        if (lambda > 0) return "unstable";
        return "nonhyperbolic";
    }

    static String saddleNodeStability(double xeq) {
        double lambda = -2.0 * xeq;
        if (lambda < 0) return "stable";
        if (lambda > 0) return "unstable";
        return "nonhyperbolic";
    }

    static double estimatePeriod(List<Double> t, List<State> z) {
        List<Double> crossings = new ArrayList<>();
        for (int k = 0; k < z.size() - 1; k++) {
            if (z.get(k).y < 0.0 && z.get(k+1).y >= 0.0) {
                double denom = z.get(k+1).y - z.get(k).y;
                if (Math.abs(denom) < 1e-14) continue;
                double alpha = -z.get(k).y / denom;
                double tc = t.get(k) + alpha * (t.get(k+1) - t.get(k));
                double xc = z.get(k).x + alpha * (z.get(k+1).x - z.get(k).x);
                if (xc > 0) crossings.add(tc);
            }
        }
        if (crossings.size() < 3) return Double.NaN;
        int start = Math.max(1, crossings.size() - 5);
        double sum = 0.0;
        int count = 0;
        for (int i = start; i < crossings.size(); i++) {
            sum += crossings.get(i) - crossings.get(i - 1);
            count++;
        }
        return (count > 0) ? sum / count : Double.NaN;
    }

    public static void main(String[] args) throws IOException {
        double mu = 1.0;
        double h = 0.01;
        double T = 80.0;
        int N = (int)(T / h);

        List<Double> t = new ArrayList<>(N + 1);
        List<State> z = new ArrayList<>(N + 1);
        t.add(0.0);
        z.add(new State(2.0, 0.1));

        for (int k = 0; k < N; k++) {
            t.add(t.get(k) + h);
            z.add(rk4Step(z.get(k), h, mu));
        }

        double amp = 0.0;
        for (int k = N/2; k < z.size(); k++) {
            amp = Math.max(amp, Math.abs(z.get(k).x));
        }

        List<Double> tTail = t.subList(N/2, t.size());
        List<State> zTail = z.subList(N/2, z.size());
        double period = estimatePeriod(tTail, zTail);

        try (FileWriter fw = new FileWriter("Chapter14_Lesson5_vdp_phase.csv")) {
            fw.write("t,x,y\n");
            for (int k = 0; k < z.size(); k++) {
                fw.write(t.get(k) + "," + z.get(k).x + "," + z.get(k).y + "\n");
            }
        }

        System.out.println("Van der Pol limit-cycle summary");
        System.out.printf("mu = %.3f%n", mu);
        System.out.printf("Amplitude |x|_max ~= %.6f%n", amp);
        System.out.printf("Period ~= %.6f%n%n", period);

        double[] rVals = {-1.0, 0.0, 1.0};
        System.out.println("Pitchfork normal form: xdot = r*x - x^3");
        for (double r : rVals) {
            System.out.print("r = " + r + ": x*=0 (" + pitchforkStability(r, 0.0) + ")");
            if (r >= 0.0) {
                double x1 = Math.sqrt(r), x2 = -Math.sqrt(r);
                System.out.print(", x*=+" + x1 + " (" + pitchforkStability(r, x1) + ")");
                System.out.print(", x*=" + x2 + " (" + pitchforkStability(r, x2) + ")");
            }
            System.out.println();
        }

        System.out.println("\nSaddle-node normal form: xdot = r - x^2");
        for (double r : rVals) {
            System.out.print("r = " + r + ": ");
            if (r < 0.0) {
                System.out.println("no real equilibria");
            } else if (r == 0.0) {
                System.out.println("x*=0 (nonhyperbolic)");
            } else {
                double x1 = Math.sqrt(r), x2 = -Math.sqrt(r);
                System.out.println("x*=+" + x1 + " (" + saddleNodeStability(x1) + "), x*=" + x2 + " (" + saddleNodeStability(x2) + ")");
            }
        }

        System.out.println("\nCSV written: Chapter14_Lesson5_vdp_phase.csv");
    }
}
