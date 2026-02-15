public final class PerformanceRequirements {
    public final double maxSettlingTime_s;
    public final double maxOvershoot;
    public final double maxRmsError;

    public PerformanceRequirements(double maxSettlingTime_s,
                                   double maxOvershoot,
                                   double maxRmsError) {
        this.maxSettlingTime_s = maxSettlingTime_s;
        this.maxOvershoot = maxOvershoot;
        this.maxRmsError = maxRmsError;
    }
}

public final class PerformanceMetrics {
    public final double settlingTime_s;
    public final double overshoot;
    public final double rmsError;

    public PerformanceMetrics(double settlingTime_s,
                              double overshoot,
                              double rmsError) {
        this.settlingTime_s = settlingTime_s;
        this.overshoot = overshoot;
        this.rmsError = rmsError;
    }
}

public final class FeasibilityReport {
    public final boolean settlingTimeOk;
    public final boolean overshootOk;
    public final boolean rmsErrorOk;

    public FeasibilityReport(boolean settlingTimeOk,
                             boolean overshootOk,
                             boolean rmsErrorOk) {
        this.settlingTimeOk = settlingTimeOk;
        this.overshootOk = overshootOk;
        this.rmsErrorOk = rmsErrorOk;
    }
}

public final class RequirementChecker {
    public static FeasibilityReport check(PerformanceRequirements req,
                                          PerformanceMetrics met) {
        boolean settlingOk = met.settlingTime_s <= req.maxSettlingTime_s;
        boolean overshootOk = met.overshoot <= req.maxOvershoot;
        boolean rmsOk = met.rmsError <= req.maxRmsError;
        return new FeasibilityReport(settlingOk, overshootOk, rmsOk);
    }

    public static void main(String[] args) {
        PerformanceRequirements req = new PerformanceRequirements(2.0, 0.05, 0.02);
        PerformanceMetrics met = new PerformanceMetrics(1.7, 0.04, 0.025);

        FeasibilityReport rep = check(req, met);
        System.out.println("settlingTimeOk = " + rep.settlingTimeOk);
        System.out.println("overshootOk    = " + rep.overshootOk);
        System.out.println("rmsErrorOk     = " + rep.rmsErrorOk);
    }
}
      
