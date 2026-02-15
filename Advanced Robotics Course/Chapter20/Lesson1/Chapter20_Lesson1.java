public class Project {
    private final String name;
    private final double I_norm;
    private final double A_norm;
    private final double D_norm;
    private final double R_norm;

    public Project(String name, double I_norm, double A_norm,
                   double D_norm, double R_norm) {
        this.name = name;
        this.I_norm = I_norm;
        this.A_norm = A_norm;
        this.D_norm = D_norm;
        this.R_norm = R_norm;
    }

    public String getName() {
        return name;
    }

    public double utility(Weights w) {
        return w.lambdaI * I_norm
             + w.lambdaA * A_norm
             - w.lambdaD * D_norm
             - w.lambdaR * R_norm;
    }

    public static class Weights {
        public double lambdaI;
        public double lambdaA;
        public double lambdaD;
        public double lambdaR;
    }

    public static void main(String[] args) {
        Project p1 = new Project("Kinodynamic RRT* with risk-sensitive cost",
                                 0.8, 0.9, 0.7, 0.6);
        Project p2 = new Project("Task and motion planning for pick-place",
                                 0.7, 0.8, 0.5, 0.4);

        Weights w = new Weights();
        w.lambdaI = 0.4;
        w.lambdaA = 0.3;
        w.lambdaD = 0.2;
        w.lambdaR = 0.1;

        Project best = p1.utility(w) > p2.utility(w) ? p1 : p2;
        System.out.println("Best project: " + best.getName());
    }
}
      
