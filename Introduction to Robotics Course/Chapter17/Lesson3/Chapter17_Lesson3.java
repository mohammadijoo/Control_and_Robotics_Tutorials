public class HomeRobotController {
    enum State { IDLE, NAVIGATE, CLEAN, DOCK, ERROR }

    private double Ts = 0.1;
    private double Kp = 1.0;
    private State state = State.IDLE;
    private double position = 0.0;
    private double goal = 0.0;
    private double battery = 1.0;
    private double coverage = 0.0;

    public void setCleaningGoal(double goalPos) {
        goal = goalPos;
        if (state == State.IDLE) {
            state = State.NAVIGATE;
        }
    }

    private boolean lowBattery() {
        return battery < 0.2;
    }

    public void step() {
        double u = 0.0;

        switch (state) {
            case IDLE:
                u = 0.0;
                break;
            case NAVIGATE:
                double e = goal - position;
                u = Kp * e;
                position += Ts * u;
                if (Math.abs(e) < 0.05) {
                    state = State.CLEAN;
                }
                if (lowBattery()) {
                    state = State.DOCK;
                }
                break;
            case CLEAN:
                u = 0.2;
                coverage = Math.min(1.0, coverage + 0.01);
                if (coverage >= 0.99) {
                    state = State.DOCK;
                }
                if (lowBattery()) {
                    state = State.DOCK;
                }
                break;
            case DOCK:
                double ed = 0.0 - position;
                u = Kp * ed;
                position += Ts * u;
                if (Math.abs(ed) < 0.05) {
                    battery = 1.0;
                    if (coverage >= 0.99) {
                        state = State.IDLE;
                    } else {
                        state = State.NAVIGATE;
                    }
                }
                break;
            case ERROR:
                u = 0.0;
                break;
        }

        battery -= Ts * (0.01 + 0.02 * Math.abs(u));
        if (battery < 0.0) battery = 0.0;
    }
}
      
