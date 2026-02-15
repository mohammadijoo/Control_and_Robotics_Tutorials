public final class Vector3 {
    public final double x, y, z;

    public Vector3(double x, double y, double z) {
        this.x = x; this.y = y; this.z = z;
    }

    public Vector3 add(Vector3 o) {
        return new Vector3(x + o.x, y + o.y, z + o.z);
    }

    public Vector3 sub(Vector3 o) {
        return new Vector3(x - o.x, y - o.y, z - o.z);
    }

    public Vector3 mul(double s) {
        return new Vector3(s * x, s * y, s * z);
    }

    public double dot(Vector3 o) {
        return x * o.x + y * o.y + z * o.z;
    }

    public double norm() {
        return Math.sqrt(this.dot(this));
    }

    public Vector3 unit() {
        double n = norm();
        if (n == 0.0) {
            throw new IllegalArgumentException("Zero vector cannot be normalized.");
        }
        return this.mul(1.0 / n);
    }

    public static double angleBetween(Vector3 a, Vector3 b) {
        Vector3 au = a.unit();
        Vector3 bu = b.unit();
        double cos = au.dot(bu);
        if (cos > 1.0) cos = 1.0;
        if (cos < -1.0) cos = -1.0;
        return Math.acos(cos);
    }
}

public final class GraspGeometry {

    public static boolean isAntipodal(
            Vector3 p1,
            Vector3 n1Out,
            Vector3 p2,
            Vector3 n2Out,
            double mu,
            double tol) {

        Vector3 n1 = n1Out.unit();
        Vector3 n2 = n2Out.unit();

        Vector3 d = p2.sub(p1).unit();
        double phi = Math.atan(mu);

        double theta1 = Vector3.angleBetween(n1.mul(-1.0), d);
        double theta2 = Vector3.angleBetween(n2, d.mul(-1.0));
        double thetaN = Vector3.angleBetween(n1, n2.mul(-1.0));

        return (theta1 <= phi + tol)
                && (theta2 <= phi + tol)
                && (thetaN >= Math.PI - 0.3);
    }

    // Typical usage in rosjava:
    // - Convert PointCloud2 contacts to Vector3.
    // - Use isAntipodal(...) to filter candidate grasps before IK.
}
      
