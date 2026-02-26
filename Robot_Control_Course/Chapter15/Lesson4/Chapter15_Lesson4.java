
public interface Learner {
    double predict(double x);
}

public class DummyLearner implements Learner {
    @Override
    public double predict(double x) {
        // Same cubic residual as in other examples
        return 0.8 * x * x * x;
    }
}

public class ScalarJointWrapper {
    private final double a;
    private final double k;
    private final double c;
    private final Learner learner;

    public ScalarJointWrapper(double a, double k, double cScale, Learner learner) {
        this.a = a;
        this.k = k;
        this.c = cScale * (a + k);
        this.learner = learner;
    }

    public double nominalControl(double x) {
        return -k * x;
    }

    public double learnedResidual(double x) {
        return learner.predict(x);
    }

    public double wrappedControl(double x) {
        double u_b = nominalControl(x);
        double u_l = learnedResidual(x);
        double bound = c * Math.abs(x);
        if (Math.abs(u_l) > bound) {
            u_l = (u_l > 0.0) ? bound : -bound;
        }
        return u_b + u_l;
    }

    public static void main(String[] args) {
        Learner learner = new DummyLearner();
        ScalarJointWrapper wrapper = new ScalarJointWrapper(1.0, 4.0, 0.5, learner);

        double x = 0.3;
        double u = wrapper.wrappedControl(x);
        System.out.println("x = " + x + ", wrapped u = " + u);
    }
}
