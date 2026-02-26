
import org.ejml.simple.SimpleMatrix;
import java.util.List;

public class ControlMetrics {

    public static double computeISE(List<Double> t,
                                    List<SimpleMatrix> e) {
        int N = t.size();
        double acc = 0.0;
        for (int k = 1; k < N; ++k) {
            double dt = t.get(k) - t.get(k - 1);
            double e2k   = e.get(k).dot(e.get(k));
            double e2km1 = e.get(k - 1).dot(e.get(k - 1));
            acc += 0.5 * dt * (e2k + e2km1);
        }
        return acc;
    }

    public static double computeISU(List<Double> t,
                                    List<SimpleMatrix> u) {
        int N = t.size();
        double acc = 0.0;
        for (int k = 1; k < N; ++k) {
            double dt = t.get(k) - t.get(k - 1);
            double u2k   = u.get(k).dot(u.get(k));
            double u2km1 = u.get(k - 1).dot(u.get(k - 1));
            acc += 0.5 * dt * (u2k + u2km1);
        }
        return acc;
    }
}
