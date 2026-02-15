public class SocialBehaviorController {

    public enum BehaviorState {
        IDLE,
        GREETING,
        SMALLTALK,
        ASSIST,
        GOODBYE
    }

    private BehaviorState state = BehaviorState.IDLE;
    private final double thetaLow;
    private final double thetaHigh;

    public SocialBehaviorController(double thetaLow, double thetaHigh) {
        this.thetaLow = thetaLow;
        this.thetaHigh = thetaHigh;
    }

    public BehaviorState getState() {
        return state;
    }

    public void step(double engagement, boolean presence) {
        switch (state) {
            case IDLE:
                if (presence && engagement < thetaHigh) {
                    state = BehaviorState.GREETING;
                }
                break;
            case GREETING:
                state = BehaviorState.SMALLTALK;
                break;
            case SMALLTALK:
                if (engagement < thetaLow) {
                    state = BehaviorState.GOODBYE;
                } else if (engagement > thetaHigh) {
                    state = BehaviorState.ASSIST;
                }
                break;
            case ASSIST:
                if (!presence || engagement < thetaLow) {
                    state = BehaviorState.GOODBYE;
                }
                break;
            case GOODBYE:
                if (!presence) {
                    state = BehaviorState.IDLE;
                }
                break;
            default:
                state = BehaviorState.IDLE;
        }
    }

    public static void main(String[] args) {
        SocialBehaviorController ctrl =
            new SocialBehaviorController(0.3, 0.8);
        double x = 0.5;
        boolean presence = true;
        for (int k = 0; k < 10; ++k) {
            ctrl.step(x, presence);
            System.out.println("k=" + k +
                               " state=" + ctrl.getState() +
                               " x=" + x +
                               " presence=" + presence);
            // here: update x and presence from sensors
        }
    }
}
      
