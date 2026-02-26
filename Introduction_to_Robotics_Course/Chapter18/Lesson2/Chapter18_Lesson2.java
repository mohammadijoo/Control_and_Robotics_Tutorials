public interface Controller {
    VelocityCommand computeAction(Observation obs);
}

public class Observation {
    public final double dFront;
    public Observation(double dFront) {
        this.dFront = dFront;
    }
}

public class VelocityCommand {
    public final double v;
    public final double w;
    public VelocityCommand(double v, double w) {
        this.v = v;
        this.w = w;
    }
}

public class ReactiveWallAvoider implements Controller {
    private final double dSafe;

    public ReactiveWallAvoider(double dSafe) {
        this.dSafe = dSafe;
    }

    @Override
    public VelocityCommand computeAction(Observation obs) {
        if (obs.dFront < dSafe) {
            return new VelocityCommand(0.0, 1.0);
        } else {
            return new VelocityCommand(0.4, 0.0);
        }
    }
}
      
