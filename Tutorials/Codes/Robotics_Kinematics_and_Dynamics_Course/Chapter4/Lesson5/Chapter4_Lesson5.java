public enum Assumption {
    IDEAL,
    OFFSET,
    LINEARIZED
}

public class Planar2RModel {
    private final double l1;
    private final double l2;
    private final double[] qOffset; // length 2

    public Planar2RModel(double l1, double l2, double[] qOffset) {
        this.l1 = l1;
        this.l2 = l2;
        this.qOffset = qOffset.clone();
    }

    public double[] fk(double[] q, Assumption assumption) {
        double q1 = q[0];
        double q2 = q[1];

        if (assumption == Assumption.OFFSET) {
            q1 += qOffset[0];
            q2 += qOffset[1];
        }

        double x, y;
        if (assumption == Assumption.LINEARIZED) {
            x = l1 + l2;
            y = l1 * q1 + l2 * (q1 + q2);
        } else {
            x = l1 * Math.cos(q1) + l2 * Math.cos(q1 + q2);
            y = l1 * Math.sin(q1) + l2 * Math.sin(q1 + q2);
        }
        return new double[]{x, y};
    }
}
      
