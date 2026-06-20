public class LocalRuleSwarm {

    static class Vec2 {
        double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 other) { return new Vec2(x + other.x, y + other.y); }
        Vec2 sub(Vec2 other) { return new Vec2(x - other.x, y - other.y); }
        Vec2 scale(double s) { return new Vec2(s * x, s * y); }
        double norm() { return Math.sqrt(x * x + y * y); }
    }

    static double phiPrime(double r, double a, double b, double eps) {
        return 2.0 * a * r - b / (r + eps);
    }

    public static void main(String[] args) {
        int N = 20;
        double a = 0.5, b = 1.0, eps = 1e-2;
        double R = 1.0, h = 0.02;
        int steps = 1000;

        Vec2[] x = new Vec2[N];
        Vec2[] xNew = new Vec2[N];

        java.util.Random rng = new java.util.Random(0);
        for (int i = 0; i < N; i++) {
            double px = -2.0 + 4.0 * rng.nextDouble();
            double py = -2.0 + 4.0 * rng.nextDouble();
            x[i] = new Vec2(px, py);
        }

        for (int k = 0; k < steps; k++) {
            for (int i = 0; i < N; i++) {
                Vec2 force = new Vec2(0.0, 0.0);
                for (int j = 0; j < N; j++) {
                    if (i == j) continue;
                    Vec2 diff = x[i].sub(x[j]);
                    double dist = diff.norm();
                    if (dist < 1e-6 || dist > R) continue;
                    double fmag = phiPrime(dist, a, b, eps);
                    Vec2 contrib = diff.scale(fmag / dist);
                    force = force.add(contrib);
                }
                xNew[i] = x[i].sub(force.scale(h));
            }
            System.arraycopy(xNew, 0, x, 0, N);
        }

        for (int i = 0; i < N; i++) {
            System.out.printf("%d : (%.3f, %.3f)%n", i, x[i].x, x[i].y);
        }
    }
}
      
