public class ActiveSensing1D {

    public interface MeasurementModel {
        double likelihood(int z, double theta, double a);
    }

    private final double[] thetaGrid;
    private double[] belief;
    private final double[] candidateViews;
    private final MeasurementModel model;

    public ActiveSensing1D(double[] thetaGrid,
                           double[] candidateViews,
                           MeasurementModel model) {
        this.thetaGrid = thetaGrid;
        this.candidateViews = candidateViews;
        this.model = model;
        this.belief = new double[thetaGrid.length];
        double init = 1.0 / thetaGrid.length;
        for (int i = 0; i < belief.length; ++i) {
            belief[i] = init;
        }
    }

    private void normalize() {
        double sum = 0.0;
        for (double v : belief) {
            sum += v;
        }
        if (sum <= 0.0) {
            double init = 1.0 / belief.length;
            for (int i = 0; i < belief.length; ++i) {
                belief[i] = init;
            }
            return;
        }
        for (int i = 0; i < belief.length; ++i) {
            belief[i] /= sum;
        }
    }

    private double entropy() {
        double H = 0.0;
        for (double b : belief) {
            if (b > 0.0) {
                H -= b * Math.log(b);
            }
        }
        return H;
    }

    private double expectedEntropyAfterAction(double a) {
        double Hexp = 0.0;
        int[] zs = {0, 1};
        for (int z : zs) {
            double[] post = new double[thetaGrid.length];
            double pz = 0.0;
            for (int i = 0; i < thetaGrid.length; ++i) {
                double like = model.likelihood(z, thetaGrid[i], a);
                post[i] = like * belief[i];
                pz += post[i];
            }
            if (pz > 0.0) {
                for (int i = 0; i < post.length; ++i) {
                    post[i] /= pz;
                }
                double Hz = 0.0;
                for (double b : post) {
                    if (b > 0.0) {
                        Hz -= b * Math.log(b);
                    }
                }
                Hexp += pz * Hz;
            }
        }
        return Hexp;
    }

    public double chooseAction() {
        double Hprior = entropy();
        double bestA = candidateViews[0];
        double bestIG = Double.NEGATIVE_INFINITY;
        for (double a : candidateViews) {
            double Hpost = expectedEntropyAfterAction(a);
            double ig = Hprior - Hpost;
            if (ig > bestIG) {
                bestIG = ig;
                bestA = a;
            }
        }
        return bestA;
    }

    public void update(double a, int z) {
        for (int i = 0; i < thetaGrid.length; ++i) {
            double like = model.likelihood(z, thetaGrid[i], a);
            belief[i] *= like;
        }
        normalize();
    }

    public double[] getBelief() {
        return belief.clone();
    }
}
      
