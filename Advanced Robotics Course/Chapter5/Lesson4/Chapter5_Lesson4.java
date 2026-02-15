public class UnicycleKinodynamics {

    public static class State {
        public double x, y, theta;
        public State(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }
    }

    public static class Control {
        public double v, w;
        public Control(double v, double w) {
            this.v = v;
            this.w = w;
        }
    }

    public static State dynamics(State s, Control u) {
        double dx = u.v * Math.cos(s.theta);
        double dy = u.v * Math.sin(s.theta);
        double dth = u.w;
        return new State(dx, dy, dth);
    }

    public static State eulerStep(State s, Control u, double dt) {
        State dx = dynamics(s, u);
        return new State(
            s.x + dt * dx.x,
            s.y + dt * dx.y,
            s.theta + dt * dx.theta
        );
    }

    public static boolean withinBounds(Control u,
                                       double vMax,
                                       double wMax) {
        if (Math.abs(u.v) > vMax) return false;
        if (Math.abs(u.w) > wMax) return false;
        return true;
    }

    public static boolean isStateValid(State s) {
        // Example: bounded workspace
        if (Math.abs(s.x) > 5.0) return false;
        if (Math.abs(s.y) > 5.0) return false;
        return true;
    }

    public static boolean simulateTrajectory(
            State x0,
            java.util.List<Control> controls,
            double dt,
            double vMax,
            double wMax,
            java.util.List<State> outTraj) {

        outTraj.clear();
        outTraj.add(x0);
        State x = x0;

        for (Control u : controls) {
            if (!withinBounds(u, vMax, wMax)) {
                return false;
            }
            State xn = eulerStep(x, u, dt);
            if (!isStateValid(xn)) {
                return false;
            }
            outTraj.add(xn);
            x = xn;
        }
        return true;
    }

    public static void main(String[] args) {
        State x0 = new State(0.0, 0.0, 0.0);
        double dt = 0.1;
        double vMax = 1.0, wMax = 2.0;

        java.util.List<Control> controls =
            new java.util.ArrayList<>();
        for (int k = 0; k < 50; ++k) {
            controls.add(new Control(0.8, 1.0));
        }

        java.util.List<State> traj =
            new java.util.ArrayList<>();
        boolean feasible = simulateTrajectory(
            x0, controls, dt, vMax, wMax, traj
        );
        System.out.println("Feasible: " + feasible);
    }
}
      
