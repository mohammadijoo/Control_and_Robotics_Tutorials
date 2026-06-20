public class ConfigMetric {

    public enum JointType { REVOLUTE, PRISMATIC }

    private static double wrapToPi(double angle) {
        double twoPi = 2.0 * Math.PI;
        angle = (angle + Math.PI) % twoPi;
        if (angle < 0.0) {
            angle += twoPi;
        }
        return angle - Math.PI;
    }

    public static double distance(double[] q1,
                                  double[] q2,
                                  JointType[] jointTypes,
                                  double[] weights) {

        int n = q1.length;
        if (q2.length != n || jointTypes.length != n || weights.length != n) {
            throw new IllegalArgumentException("Dimension mismatch");
        }

        double sumSq = 0.0;
        for (int i = 0; i < n; ++i) {
            double delta = q2[i] - q1[i];
            if (jointTypes[i] == JointType.REVOLUTE) {
                delta = wrapToPi(delta);
            }
            double wdelta = weights[i] * delta;
            sumSq += wdelta * wdelta;
        }
        return Math.sqrt(sumSq);
    }

    public static void main(String[] args) {
        double[] q1 = {0.0, 0.5, 0.1};
        double[] q2 = {Math.PI - 0.2, 0.6, 0.05};
        JointType[] jointTypes = {
            JointType.REVOLUTE,
            JointType.REVOLUTE,
            JointType.PRISMATIC
        };
        double[] weights = {1.0 / Math.PI, 1.0 / Math.PI, 2.0};

        double d = distance(q1, q2, jointTypes, weights);
        System.out.println("Configuration distance = " + d);
    }
}
      
