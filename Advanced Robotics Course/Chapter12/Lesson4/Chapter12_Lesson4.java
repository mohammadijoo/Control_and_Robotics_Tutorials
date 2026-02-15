public class ReachTaskEnv {

    public static class Obs {
        public double[] eePos;   // length 3
        public double[] goal;    // length 3
        public boolean success;
    }

    private double alpha = 1.0;
    private double ctrlCoeff = 0.01;

    private double distance(double[] a, double[] b) {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private double baseReward(Obs s, Obs sNext, double[] action) {
        double dist = distance(s.eePos, s.goal);
        double rTask = -alpha * dist;

        double normSq = 0.0;
        for (double v : action) {
            normSq += v * v;
        }
        double rCtrl = -ctrlCoeff * normSq;

        double rTerm = sNext.success ? 1.0 : 0.0;
        return rTask + rCtrl + rTerm;
    }

    public double stepReward(Obs s, Obs sNext, double[] action) {
        return baseReward(s, sNext, action);
    }
}
      
