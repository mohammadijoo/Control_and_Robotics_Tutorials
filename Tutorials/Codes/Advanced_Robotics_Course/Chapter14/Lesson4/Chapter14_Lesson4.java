public class Agent {
    public double[] p = new double[2];
    public double[] v = new double[2];
    public double[] vPref = new double[2];
    public double r;
    public double maxSpeed;

    public Agent(double[] p0, double[] v0, double r, double[] vPref, double maxSpeed) {
        this.p[0] = p0[0]; this.p[1] = p0[1];
        this.v[0] = v0[0]; this.v[1] = v0[1];
        this.r = r;
        this.vPref[0] = vPref[0]; this.vPref[1] = vPref[1];
        this.maxSpeed = maxSpeed;
    }
}

class HalfPlane {
    public double[] n = new double[2];
    public double[] p0 = new double[2]; // (v - p0) . n >= 0
}

public class ORCA2D {

    private static double dot(double[] a, double[] b) {
        return a[0]*b[0] + a[1]*b[1];
    }

    private static double norm(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    private static void sub(double[] out, double[] a, double[] b) {
        out[0] = a[0] - b[0];
        out[1] = a[1] - b[1];
    }

    private static void add(double[] out, double[] a, double[] b) {
        out[0] = a[0] + b[0];
        out[1] = a[1] + b[1];
    }

    private static void scale(double[] out, double[] a, double s) {
        out[0] = a[0] * s;
        out[1] = a[1] * s;
    }

    private static void normalize(double[] out, double[] a) {
        double n = norm(a);
        if (n > 1e-9) {
            out[0] = a[0] / n;
            out[1] = a[1] / n;
        } else {
            out[0] = a[0];
            out[1] = a[1];
        }
    }

    private static double timeToCollision(double[] p_ij,
                                          double[] v_ij,
                                          double R, double T_h)
    {
        double a = dot(v_ij, v_ij);
        double c = dot(p_ij, p_ij) - R*R;
        if (c <= 0.0) return 0.0;
        if (Math.abs(a) < 1e-9) return Double.POSITIVE_INFINITY;
        double b = 2.0 * dot(p_ij, v_ij);
        double disc = b*b - 4.0*a*c;
        if (disc < 0.0) return Double.POSITIVE_INFINITY;
        double tMin = (-b - Math.sqrt(disc)) / (2.0*a);
        if (tMin < 0.0 || tMin > T_h)
            return Double.POSITIVE_INFINITY;
        return tMin;
    }

    private static java.util.List<HalfPlane> orcaConstraintsForAgent(
            int i,
            java.util.List<Agent> agents,
            double T_h,
            double delta)
    {
        Agent ai = agents.get(i);
        java.util.List<HalfPlane> cons = new java.util.ArrayList<>();

        for (int j = 0; j < agents.size(); ++j) {
            if (j == i) continue;
            Agent aj = agents.get(j);

            double[] p_ij = new double[2];
            double[] v_ij = new double[2];
            sub(p_ij, aj.p, ai.p);
            sub(v_ij, ai.v, aj.v);

            double R = ai.r + aj.r;
            double ttc = timeToCollision(p_ij, v_ij, R, T_h);
            if (!Double.isFinite(ttc)) continue;

            double[] pColl = new double[2];
            double[] tmp = new double[2];
            scale(tmp, v_ij, ttc);
            add(pColl, p_ij, tmp);

            double[] n_ij = new double[2];
            normalize(n_ij, pColl);

            double[] u_ij = new double[2];
            scale(tmp, n_ij, R);
            sub(tmp, tmp, pColl); // R * n_ij - pColl
            double denom = Math.max(ttc, delta);
            scale(u_ij, tmp, 1.0 / denom);

            HalfPlane hp = new HalfPlane();
            hp.n[0] = n_ij[0];
            hp.n[1] = n_ij[1];
            hp.p0[0] = ai.v[0] + 0.5 * u_ij[0];
            hp.p0[1] = ai.v[1] + 0.5 * u_ij[1];
            cons.add(hp);
        }
        return cons;
    }

    private static double[] projectVelocity(double[] vPref,
                                            java.util.List<HalfPlane> cons,
                                            double maxSpeed)
    {
        double[] v = new double[]{vPref[0], vPref[1]};
        double nrm = norm(v);
        if (nrm > maxSpeed) {
            v[0] *= maxSpeed / nrm;
            v[1] *= maxSpeed / nrm;
        }

        for (HalfPlane hp : cons) {
            double[] diff = new double[]{v[0] - hp.p0[0], v[1] - hp.p0[1]};
            double val = dot(diff, hp.n);
            if (val < 0.0) {
                double alpha = dot(new double[]{hp.p0[0] - v[0], hp.p0[1] - v[1]}, hp.n);
                v[0] += alpha * hp.n[0];
                v[1] += alpha * hp.n[1];

                nrm = norm(v);
                if (nrm > maxSpeed) {
                    v[0] *= maxSpeed / nrm;
                    v[1] *= maxSpeed / nrm;
                }
            }
        }
        return v;
    }

    public static void stepORCA(java.util.List<Agent> agents,
                                double T_h, double delta)
    {
        java.util.List<double[]> newV = new java.util.ArrayList<>(agents.size());
        for (int i = 0; i < agents.size(); ++i) {
            java.util.List<HalfPlane> cons =
                    orcaConstraintsForAgent(i, agents, T_h, delta);
            double[] vStar = projectVelocity(agents.get(i).vPref, cons,
                                             agents.get(i).maxSpeed);
            newV.add(vStar);
        }
        // synchronous update
        for (int i = 0; i < agents.size(); ++i) {
            Agent a = agents.get(i);
            double[] vStar = newV.get(i);
            a.v[0] = vStar[0];
            a.v[1] = vStar[1];
            a.p[0] += delta * a.v[0];
            a.p[1] += delta * a.v[1];
        }
    }
}
      
