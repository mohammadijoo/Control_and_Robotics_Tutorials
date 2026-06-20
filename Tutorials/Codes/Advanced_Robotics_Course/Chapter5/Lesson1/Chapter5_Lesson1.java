public class DoubleIntegrator {

    public static class State {
        public double x;
        public double v;
        public State(double x, double v) {
            this.x = x;
            this.v = v;
        }
    }

    public static java.util.List<State> propagate(
            State x0, double u, double dt, double T_step, double uMax) {

        double uSat = Math.max(-uMax, Math.min(u, uMax));
        State x = new State(x0.x, x0.v);
        java.util.List<State> traj = new java.util.ArrayList<>();
        traj.add(new State(x.x, x.v));

        int N = (int)(T_step / dt);
        for (int k = 0; k < N; ++k) {
            double xdot = x.v;
            double vdot = uSat;

            x.x += dt * xdot;
            x.v += dt * vdot;

            traj.add(new State(x.x, x.v));
        }
        return traj;
    }

    public static void main(String[] args) {
        State x0 = new State(0.0, 0.0);
        double u = 0.8;
        java.util.List<State> trajectory =
            propagate(x0, u, 0.01, 0.5, 1.0);

        State xf = trajectory.get(trajectory.size() - 1);
        System.out.println("Final state: x = " + xf.x + ", v = " + xf.v);
    }
}
      
