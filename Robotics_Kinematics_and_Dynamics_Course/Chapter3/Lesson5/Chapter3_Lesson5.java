public class Quaternion {
    public double w, x, y, z;

    public Quaternion(double w, double x, double y, double z) {
        this.w = w; this.x = x; this.y = y; this.z = z;
    }

    public double dot(Quaternion other) {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    public void normalize() {
        double n = Math.sqrt(w*w + x*x + y*y + z*z);
        w /= n; x /= n; y /= n; z /= n;
    }

    public static Quaternion continuous(Quaternion qPrev, Quaternion qRaw) {
        Quaternion qNew = new Quaternion(qRaw.w, qRaw.x, qRaw.y, qRaw.z);
        if (qPrev.dot(qNew) < 0.0) {
            qNew.w = -qNew.w;
            qNew.x = -qNew.x;
            qNew.y = -qNew.y;
            qNew.z = -qNew.z;
        }
        qNew.normalize();
        return qNew;
    }
}
      
