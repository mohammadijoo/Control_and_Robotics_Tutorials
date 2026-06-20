public class Robot {
    private final int id;
    private final double[] values;
    private final double epsilon;
    private int assignedTask = -1;

    public Robot(int id, double[] values, double epsilon) {
        this.id = id;
        this.values = values;
        this.epsilon = epsilon;
    }

    public boolean isUnassigned() {
        return assignedTask < 0;
    }

    public Bid computeBid(double[] prices) {
        double best = Double.NEGATIVE_INFINITY;
        double second = Double.NEGATIVE_INFINITY;
        int j1 = -1;
        for (int j = 0; j < values.length; ++j) {
            double reduced = values[j] - prices[j];
            if (reduced > best) {
                second = best;
                best = reduced;
                j1 = j;
            } else if (reduced > second) {
                second = reduced;
            }
        }
        double amount = prices[j1] + (best - second) + epsilon;
        return new Bid(j1, amount, id);
    }

    public static class Bid {
        public final int task;
        public final double amount;
        public final int robotId;
        public Bid(int task, double amount, int robotId) {
            this.task = task;
            this.amount = amount;
            this.robotId = robotId;
        }
    }
}
      
