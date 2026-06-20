import java.util.ArrayList;
import java.util.List;

class Sample {
    double[] s; // length 4
    double[] a; // length 2
    Sample(double[] s, double[] a) {
        this.s = s;
        this.a = a;
    }
}

class LinearPolicy {
    // a = W s + b, W: 2x4, b: 2
    double[][] W = new double[2][4];
    double[] b = new double[2];

    double[] act(double[] s) {
        double[] a = new double[2];
        for (int i = 0; i < 2; i++) {
            a[i] = b[i];
            for (int j = 0; j < 4; j++) {
                a[i] += W[i][j] * s[j];
            }
        }
        return a;
    }
}

public class DAggerDemo {

    static double[] expertPolicy(double[] s) {
        double[] q_des = {0.5, -0.5};
        double[] q = {s[0], s[1]};
        double[] dq = {s[2], s[3]};
        double Kp = 5.0, Kd = 1.0;
        double[] tau = new double[2];
        for (int i = 0; i < 2; i++) {
            tau[i] = Kp * (q_des[i] - q[i]) - Kd * dq[i];
        }
        return tau;
    }

    static double[] simulate(double[] s, double[] a, double dt) {
        double q1 = s[0], q2 = s[1];
        double dq1 = s[2], dq2 = s[3];
        double ddq1 = a[0], ddq2 = a[1];
        double dq1n = dq1 + ddq1 * dt;
        double dq2n = dq2 + ddq2 * dt;
        double q1n = q1 + dq1n * dt;
        double q2n = q2 + dq2n * dt;
        return new double[]{q1n, q2n, dq1n, dq2n};
    }

    static List<Sample> collectExpertRollout(int T) {
        List<Sample> traj = new ArrayList<>();
        double[] s = {0, 0, 0, 0};
        for (int t = 0; t < T; t++) {
            double[] a = expertPolicy(s);
            traj.add(new Sample(s.clone(), a.clone()));
            s = simulate(s, a, 0.02);
        }
        return traj;
    }

    static void trainBC(LinearPolicy pi, List<Sample> data, int iters, double lr) {
        for (int it = 0; it < iters; it++) {
            double[][] gradW = new double[2][4];
            double[] gradb = new double[2];
            for (Sample sample : data) {
                double[] pred = pi.act(sample.s);
                double[] err = {pred[0] - sample.a[0], pred[1] - sample.a[1]};
                for (int i = 0; i < 2; i++) {
                    gradb[i] += err[i];
                    for (int j = 0; j < 4; j++) {
                        gradW[i][j] += err[i] * sample.s[j];
                    }
                }
            }
            int N = data.size();
            for (int i = 0; i < 2; i++) {
                gradb[i] /= N;
                for (int j = 0; j < 4; j++) gradW[i][j] /= N;
            }
            for (int i = 0; i < 2; i++) {
                pi.b[i] -= lr * gradb[i];
                for (int j = 0; j < 4; j++) {
                    pi.W[i][j] -= lr * gradW[i][j];
                }
            }
        }
    }

    static void daggerTrain() {
        int numIters = 5;
        int T = 200;
        LinearPolicy pi = new LinearPolicy();
        List<Sample> dataset = collectExpertRollout(T);

        for (int i = 0; i < numIters; i++) {
            trainBC(pi, dataset, 200, 1e-3);
            List<Sample> newSamples = new ArrayList<>();
            double[] s = {0, 0, 0, 0};
            for (int t = 0; t < T; t++) {
                double[] a_pi = pi.act(s);
                double[] a_exp = expertPolicy(s);
                newSamples.add(new Sample(s.clone(), a_exp.clone()));
                s = simulate(s, a_pi, 0.02);
            }
            dataset.addAll(newSamples);
            System.out.println("DAgger iter " + (i + 1) + ", dataset size = " + dataset.size());
        }
    }

    public static void main(String[] args) {
        daggerTrain();
    }
}
      
