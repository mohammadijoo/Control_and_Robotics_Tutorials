public final class Quaternion {
    public final double w, x, y, z;

    public Quaternion(double w, double x, double y, double z) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }

    public static Quaternion fromAxisAngle(double[] axis, double theta) {
        double norm = Math.sqrt(axis[0]*axis[0] +
                                axis[1]*axis[1] +
                                axis[2]*axis[2]);
        double ux = axis[0] / norm;
        double uy = axis[1] / norm;
        double uz = axis[2] / norm;
        double half = 0.5 * theta;
        double s = Math.sin(half);
        return new Quaternion(Math.cos(half), ux*s, uy*s, uz*s);
    }

    public Quaternion normalized() {
        double n = Math.sqrt(w*w + x*x + y*y + z*z);
        return new Quaternion(w / n, x / n, y / n, z / n);
    }

    public Quaternion conjugate() {
        return new Quaternion(w, -x, -y, -z);
    }

    public Quaternion multiply(Quaternion q) {
        double nw = w*q.w - x*q.x - y*q.y - z*q.z;
        double nx = w*q.x + x*q.w + y*q.z - z*q.y;
        double ny = w*q.y - x*q.z + y*q.w + z*q.x;
        double nz = w*q.z + x*q.y - y*q.x + z*q.w;
        return new Quaternion(nw, nx, ny, nz);
    }

    public double dot(Quaternion q) {
        return w*q.w + x*q.x + y*q.y + z*q.z;
    }

    public static Quaternion slerp(Quaternion q0_in,
                                   Quaternion q1_in,
                                   double t) {
        Quaternion q0 = q0_in.normalized();
        Quaternion q1 = q1_in.normalized();
        double dot = q0.dot(q1);

        // Ensure shortest arc
        if (dot < 0.0) {
            q1 = new Quaternion(-q1.w, -q1.x, -q1.y, -q1.z);
            dot = -dot;
        }

        dot = Math.max(-1.0, Math.min(1.0, dot));
        double theta = Math.acos(dot);

        if (theta < 1e-6) {
            // Nlerp fallback
            double w = (1.0 - t)*q0.w + t*q1.w;
            double x = (1.0 - t)*q0.x + t*q1.x;
            double y = (1.0 - t)*q0.y + t*q1.y;
            double z = (1.0 - t)*q0.z + t*q1.z;
            return new Quaternion(w, x, y, z).normalized();
        }

        double sinTheta = Math.sin(theta);
        double w0 = Math.sin((1.0 - t)*theta) / sinTheta;
        double w1 = Math.sin(t*theta) / sinTheta;

        Quaternion q = new Quaternion(
            w0*q0.w + w1*q1.w,
            w0*q0.x + w1*q1.x,
            w0*q0.y + w1*q1.y,
            w0*q0.z + w1*q1.z
        );
        return q.normalized();
    }

    public double[] rotateVector(double[] v) {
        // Convert v into a pure quaternion and apply q v q^{-1}
        Quaternion vq = new Quaternion(0.0, v[0], v[1], v[2]);
        Quaternion q = this.normalized();
        Quaternion res = q.multiply(vq).multiply(q.conjugate());
        return new double[]{res.x, res.y, res.z};
    }
}
      
