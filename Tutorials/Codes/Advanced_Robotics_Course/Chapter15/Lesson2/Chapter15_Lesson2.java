public class Vec2 {
    public double x, y;
    public Vec2(double x, double y) { this.x = x; this.y = y; }
    public Vec2 plus(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
    public Vec2 minus(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
    public Vec2 times(double s) { return new Vec2(x * s, y * s); }
    public double norm() { return Math.sqrt(x * x + y * y); }
}

public class Swarm2D {
    public enum Behavior { AGGREGATE, DISPERSE }
    private int N;
    private double dt;
    private Behavior behavior;
    private Vec2[] x;
    private double R = 0.4;
    private double kAgg = 1.0;
    private double kDisp = 0.5;
    private double dDes = 0.2;

    public Swarm2D(int N, double dt, Behavior b) {
        this.N = N; this.dt = dt; this.behavior = b;
        this.x = new Vec2[N];
        java.util.Random rand = new java.util.Random(42);
        for (int i = 0; i < N; ++i) {
            x[i] = new Vec2(rand.nextDouble(), rand.nextDouble());
        }
    }

    private void neighbors(int i,
                           java.util.List<Integer> idx,
                           java.util.List<Vec2> diffs,
                           java.util.List<Double> dists) {
        idx.clear(); diffs.clear(); dists.clear();
        for (int j = 0; j < N; ++j) {
            if (j == i) continue;
            Vec2 d = x[j].minus(x[i]);
            double r = d.norm();
            if (r > 0.0 && r < R) {
                idx.add(j);
                diffs.add(d);
                dists.add(r);
            }
        }
    }

    public void step() {
        switch (behavior) {
            case AGGREGATE: stepAggregate(); break;
            case DISPERSE: stepDispersion(); break;
        }
    }

    private void stepAggregate() {
        Vec2[] u = new Vec2[N];
        java.util.List<Integer> idx = new java.util.ArrayList<>();
        java.util.List<Vec2> diffs = new java.util.ArrayList<>();
        java.util.List<Double> dists = new java.util.ArrayList<>();
        for (int i = 0; i < N; ++i) {
            neighbors(i, idx, diffs, dists);
            Vec2 ui = new Vec2(0.0, 0.0);
            for (Vec2 d : diffs) ui = ui.plus(d);
            u[i] = ui.times(-kAgg);
        }
        for (int i = 0; i < N; ++i) {
            x[i] = x[i].plus(u[i].times(dt));
        }
    }

    private void stepDispersion() {
        Vec2[] u = new Vec2[N];
        java.util.List<Integer> idx = new java.util.ArrayList<>();
        java.util.List<Vec2> diffs = new java.util.ArrayList<>();
        java.util.List<Double> dists = new java.util.ArrayList<>();
        for (int i = 0; i < N; ++i) {
            neighbors(i, idx, diffs, dists);
            Vec2 ui = new Vec2(0.0, 0.0);
            for (int k = 0; k < idx.size(); ++k) {
                Vec2 d = diffs.get(k);
                double r = dists.get(k);
                double r2 = r * r;
                double phiPrime = (r2 - dDes * dDes) * r;
                Vec2 dir = new Vec2(d.x / r, d.y / r);
                ui = ui.plus(dir.times(-phiPrime));
            }
            u[i] = ui.times(kDisp);
        }
        for (int i = 0; i < N; ++i) {
            x[i] = x[i].plus(u[i].times(dt));
        }
    }

    public Vec2[] getPositions() { return x; }
}
      
