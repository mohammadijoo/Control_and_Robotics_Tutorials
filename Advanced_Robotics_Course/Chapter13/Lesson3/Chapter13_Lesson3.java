import java.util.*;
import java.util.stream.*;

class Trajectory {
    public double[][] q;
    public double[][] qd;
    public double[][] eePos;
}

class Experiment {
    public Trajectory realTraj;
    // plus commands, initial state, etc.
}

interface SimulatorInterface {
    Trajectory rolloutSim(double[] theta, Experiment exp);
}

public class CalibrationManager {

    private final SimulatorInterface sim;
    private final List<Experiment> experiments;
    private final Random rng = new Random(0L);

    public CalibrationManager(SimulatorInterface sim,
                              List<Experiment> experiments) {
        this.sim = sim;
        this.experiments = experiments;
    }

    private double[] sampleTheta() {
        double[] theta = new double[3];
        for (int i = 0; i < 3; ++i) {
            theta[i] = 0.5 + rng.nextDouble(); // [0.5, 1.5]
        }
        return theta;
    }

    private double[] trajectoryFeatures(Trajectory traj) {
        List<double[]> all = new ArrayList<>();
        all.addAll(Arrays.asList(traj.q));
        all.addAll(Arrays.asList(traj.qd));
        all.addAll(Arrays.asList(traj.eePos));

        List<Double> feats = new ArrayList<>();
        int dim = all.get(0).length;
        int T = traj.q.length;

        for (int k = 0; k < all.size(); ++k) {
            double[] seq = all.get(k);
        }

        // Means and stds for each coordinate across q, qd, eePos
        for (double[][] seqGroup : new double[][][]{traj.q, traj.qd, traj.eePos}) {
            int dimGroup = seqGroup[0].length;
            double[] mean = new double[dimGroup];
            double[] sq = new double[dimGroup];
            int Tgroup = seqGroup.length;
            for (int t = 0; t < Tgroup; ++t) {
                double[] x = seqGroup[t];
                for (int i = 0; i < dimGroup; ++i) {
                    mean[i] += x[i];
                    sq[i] += x[i] * x[i];
                }
            }
            for (int i = 0; i < dimGroup; ++i) {
                mean[i] /= Tgroup;
                double var = sq[i] / Tgroup - mean[i] * mean[i];
                double stdv = Math.sqrt(Math.max(var, 0.0));
                feats.add(mean[i]);
                feats.add(stdv);
            }
        }

        return feats.stream().mapToDouble(d -> d).toArray();
    }

    private double featureMismatch(Trajectory realTraj,
                                   Trajectory simTraj) {
        double[] fr = trajectoryFeatures(realTraj);
        double[] fs = trajectoryFeatures(simTraj);
        double cost = 0.0;
        for (int i = 0; i < fr.length; ++i) {
            double d = fr[i] - fs[i];
            cost += d * d;
        }
        return cost;
    }

    private double calibrationObjective(double[] theta) {
        double total = 0.0;
        for (Experiment e : experiments) {
            Trajectory simTraj = sim.rolloutSim(theta, e);
            total += featureMismatch(e.realTraj, simTraj);
        }
        return total;
    }

    public double[] randomSearch(int numSamples) {
        double bestCost = Double.POSITIVE_INFINITY;
        double[] bestTheta = null;
        for (int k = 0; k < numSamples; ++k) {
            double[] theta = sampleTheta();
            double cost = calibrationObjective(theta);
            if (cost < bestCost) {
                bestCost = cost;
                bestTheta = theta;
            }
        }
        return bestTheta;
    }
}
      
