public class StateLatticePlanner {

    static final double DX = 1.0;
    static final double DY = 1.0;
    static final int N_TH = 16;
    static final double DTH = 2.0 * Math.PI / N_TH;

    static class LatticeState {
        int i, j, k;
        LatticeState(int i, int j, int k) { this.i = i; this.j = j; this.k = k; }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof LatticeState)) return false;
            LatticeState other = (LatticeState)o;
            return i == other.i && j == other.j && k == other.k;
        }

        @Override
        public int hashCode() {
            return (i * 73856093) ^ (j * 19349663) ^ (k * 83492791);
        }
    }

    static class Primitive {
        double v, omega, T, dt;
        double dx, dy, dth;
        double cost;

        Primitive(double v, double omega, double T, double dt) {
            this.v = v;
            this.omega = omega;
            this.T = T;
            this.dt = dt;
            simulateCanonical();
        }

        private void simulateCanonical() {
            double x = 0.0, y = 0.0, th = 0.0;
            double t = 0.0;
            double c = 0.0;
            while (t < T) {
                x += v * Math.cos(th) * dt;
                y += v * Math.sin(th) * dt;
                th += omega * dt;
                t += dt;
                c += dt;
            }
            th = wrapTheta(th);
            dx = x; dy = y; dth = th;
            cost = c;
        }
    }

    static double wrapTheta(double th) {
        double twoPi = 2.0 * Math.PI;
        th = th % twoPi;
        if (th < 0.0) th += twoPi;
        return th;
    }

    static LatticeState quantize(double x, double y, double th) {
        int i = (int)Math.round(x / DX);
        int j = (int)Math.round(y / DY);
        int k = ((int)Math.round(wrapTheta(th) / DTH)) % N_TH;
        return new LatticeState(i, j, k);
    }

    static double heuristic(LatticeState a, LatticeState b) {
        double dx = (a.i - b.i) * DX;
        double dy = (a.j - b.j) * DY;
        double dist = Math.hypot(dx, dy);
        double vMax = 1.0;
        return dist / vMax;
    }

    // Here one would add a priority-queue based A* using Primitive successors.

    public static void main(String[] args) {
        // Example primitive set
        Primitive[] prims = {
            new Primitive(1.0, 0.0, 1.0, 0.1),
            new Primitive(1.0, 0.8, 1.0, 0.1),
            new Primitive(1.0, -0.8, 1.0, 0.1)
        };
        // A* implementation would use these primitives to expand neighbors.
    }
}
      
