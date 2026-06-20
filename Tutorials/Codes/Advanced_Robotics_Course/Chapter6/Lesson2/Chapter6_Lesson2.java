public class GaussianBelief1D {
    private double mu;
    private double sigma2;

    public GaussianBelief1D(double mu, double sigma2) {
        this.mu = mu;
        this.sigma2 = sigma2;
    }

    public double getMu() {
        return mu;
    }

    public double getSigma2() {
        return sigma2;
    }

    public GaussianBelief1D predict(double u, double sigmaW2) {
        double muPred = mu + u;
        double sigma2Pred = sigma2 + sigmaW2;
        return new GaussianBelief1D(muPred, sigma2Pred);
    }

    public GaussianBelief1D update(double z, double sigmaV2) {
        double K = sigma2 / (sigma2 + sigmaV2);
        double muPost = mu + K * (z - mu);
        double sigma2Post = (1.0 - K) * sigma2;
        return new GaussianBelief1D(muPost, sigma2Post);
    }

    public static double oneStepCost(GaussianBelief1D b,
                                     double u,
                                     double xGoal,
                                     double sigmaW2,
                                     double lambdaUnc) {
        GaussianBelief1D pred = b.predict(u, sigmaW2);
        double muErr = pred.getMu() - xGoal;
        return muErr * muErr + lambdaUnc * pred.getSigma2();
    }

    public static double chooseControl(GaussianBelief1D b,
                                       double[] actions,
                                       double xGoal,
                                       double sigmaW2,
                                       double lambdaUnc) {
        double bestU = actions[0];
        double bestCost = oneStepCost(b, bestU, xGoal, sigmaW2, lambdaUnc);
        for (int i = 1; i < actions.length; ++i) {
            double u = actions[i];
            double cost = oneStepCost(b, u, xGoal, sigmaW2, lambdaUnc);
            if (cost < bestCost) {
                bestCost = cost;
                bestU = u;
            }
        }
        return bestU;
    }

    public static void main(String[] args) {
        GaussianBelief1D belief = new GaussianBelief1D(0.0, 1.0);
        double[] actions = {-1.0, 0.0, 1.0};
        double xGoal = 5.0;
        double sigmaW2 = 0.1;
        double lambdaUnc = 0.5;

        double uStar = chooseControl(belief, actions, xGoal, sigmaW2, lambdaUnc);
        System.out.println("Chosen control: " + uStar);
    }
}
      
