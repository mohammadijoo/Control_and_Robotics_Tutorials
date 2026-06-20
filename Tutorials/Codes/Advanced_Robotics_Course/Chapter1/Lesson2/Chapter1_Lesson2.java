public final class PlanarArmCollision {

    static class Vec2 {
        final double x, y;
        Vec2(double x, double y) { this.x = x; this.y = y; }
        Vec2 add(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
        Vec2 sub(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
        Vec2 scale(double s) { return new Vec2(s * x, s * y); }
        double dot(Vec2 o) { return x * o.x + y * o.y; }
        double norm() { return Math.sqrt(dot(this)); }
    }

    static Vec2[] forwardKinematics(double q1, double q2,
                                    double L1, double L2) {
        Vec2 p0 = new Vec2(0.0, 0.0);
        Vec2 p1 = new Vec2(L1 * Math.cos(q1), L1 * Math.sin(q1));
        Vec2 p2 = p1.add(new Vec2(
                L2 * Math.cos(q1 + q2),
                L2 * Math.sin(q1 + q2)));
        return new Vec2[]{p0, p1, p2};
    }

    static double segmentPointDistance(Vec2 a, Vec2 b, Vec2 p) {
        Vec2 ab = b.sub(a);
        double denom = ab.dot(ab) + 1e-12;
        double t = p.sub(a).dot(ab) / denom;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        Vec2 proj = a.add(ab.scale(t));
        return p.sub(proj).norm();
    }

    static boolean inCollision(double q1, double q2,
                               Vec2 center, double radius) {
        Vec2[] pts = forwardKinematics(q1, q2, 1.0, 1.0);
        double d1 = segmentPointDistance(pts[0], pts[1], center);
        double d2 = segmentPointDistance(pts[1], pts[2], center);
        return (d1 <= radius) || (d2 <= radius);
    }

    public static void main(String[] args) {
        Vec2 center = new Vec2(0.8, 0.4);
        double radius = 0.25;
        double q1 = 0.3, q2 = -1.0;
        boolean coll = inCollision(q1, q2, center, radius);
        System.out.println("Collision? " + coll);
    }
}
      
