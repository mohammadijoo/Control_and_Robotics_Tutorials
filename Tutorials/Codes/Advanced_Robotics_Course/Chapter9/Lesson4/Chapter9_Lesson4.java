public class TAMPObjective {

    private int T;
    private double[] qStart;
    private double[] qGoal;
    private String[] actions; // length T, e.g. "move", "grasp"

    public TAMPObjective(int T, double[] qStart, double[] qGoal, String[] actions) {
        this.T = T;
        this.qStart = qStart;
        this.qGoal = qGoal;
        this.actions = actions;
    }

    // traj is flat array of length 2*(T+1): [q0x,q0y,q1x,q1y,...]
    public double evaluate(double[] traj) {
        double cost = 0.0;

        // smoothness
        for (int k = 0; k < T; ++k) {
            double qkx = traj[2 * k];
            double qky = traj[2 * k + 1];
            double qkx1 = traj[2 * (k + 1)];
            double qky1 = traj[2 * (k + 1) + 1];
            double dx = qkx1 - qkx;
            double dy = qky1 - qky;
            cost += dx * dx + dy * dy;
        }

        // goal
        double qTx = traj[2 * T];
        double qTy = traj[2 * T + 1];
        double dxg = qTx - qGoal[0];
        double dyg = qTy - qGoal[1];
        cost += dxg * dxg + dyg * dyg;

        // simple "grasp" cost
        for (int k = 0; k < T; ++k) {
            if (actions[k].equals("grasp")) {
                double qkx = traj[2 * k];
                double qky = traj[2 * k + 1];
                double objx = 0.5;
                double objy = 0.5;
                double dx = qkx - objx;
                double dy = qky - objy;
                cost += 10.0 * (dx * dx + dy * dy);
            }
        }
        return cost;
    }
}
      
