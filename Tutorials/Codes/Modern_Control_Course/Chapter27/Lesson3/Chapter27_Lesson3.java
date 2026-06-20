// Chapter27_Lesson3.java
// From-scratch RK4 simulation of the internal-model servo.
// Compile: javac Chapter27_Lesson3.java
// Run:     java Chapter27_Lesson3

public class Chapter27_Lesson3 {
    static class State {
        double x1;
        double x2;
        double eta;

        State(double x1, double x2, double eta) {
            this.x1 = x1;
            this.x2 = x2;
            this.eta = eta;
        }
    }

    static State addScaled(State a, State b, double h) {
        return new State(a.x1 + h * b.x1, a.x2 + h * b.x2, a.eta + h * b.eta);
    }

    static State rhs(State s, double reference) {
        double y = s.x1;
        double u = -26.0 * s.x1 - 8.6 * s.x2 + 40.0 * s.eta;

        double dx1 = s.x2;
        double dx2 = -2.0 * s.x1 - 0.4 * s.x2 + u;
        double deta = reference - y;
        return new State(dx1, dx2, deta);
    }

    static State rk4Step(State s, double h, double reference) {
        State k1 = rhs(s, reference);
        State k2 = rhs(addScaled(s, k1, 0.5 * h), reference);
        State k3 = rhs(addScaled(s, k2, 0.5 * h), reference);
        State k4 = rhs(addScaled(s, k3, h), reference);

        double x1 = s.x1 + h * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1) / 6.0;
        double x2 = s.x2 + h * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2) / 6.0;
        double eta = s.eta + h * (k1.eta + 2.0 * k2.eta + 2.0 * k3.eta + k4.eta) / 6.0;
        return new State(x1, x2, eta);
    }

    public static void main(String[] args) {
        double reference = 1.0;
        double h = 0.001;
        double tf = 8.0;
        int steps = (int) Math.round(tf / h);

        State s = new State(0.0, 0.0, 0.0);
        double peakAbsU = 0.0;

        for (int k = 0; k < steps; k++) {
            double u = -26.0 * s.x1 - 8.6 * s.x2 + 40.0 * s.eta;
            peakAbsU = Math.max(peakAbsU, Math.abs(u));
            s = rk4Step(s, h, reference);
        }

        double y = s.x1;
        double error = reference - y;

        System.out.printf("Final output y(T) = %.8f%n", y);
        System.out.printf("Final tracking error e(T) = %.8e%n", error);
        System.out.printf("Final integrator state eta(T) = %.8f%n", s.eta);
        System.out.printf("Peak absolute control = %.8f%n", peakAbsU);
    }
}
