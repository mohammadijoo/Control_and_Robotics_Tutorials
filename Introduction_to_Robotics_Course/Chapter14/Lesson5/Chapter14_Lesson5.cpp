#include <iostream>

enum class BehaviorState {
    IDLE,
    GREETING,
    SMALLTALK,
    ASSIST,
    GOODBYE
};

BehaviorState step_fsm(BehaviorState s,
                       double x,
                       bool presence,
                       double theta_low,
                       double theta_high)
{
    switch (s) {
    case BehaviorState::IDLE:
        if (presence && x < theta_high)
            return BehaviorState::GREETING;
        return s;
    case BehaviorState::GREETING:
        return BehaviorState::SMALLTALK;
    case BehaviorState::SMALLTALK:
        if (x < theta_low)
            return BehaviorState::GOODBYE;
        if (x > theta_high)
            return BehaviorState::ASSIST;
        return s;
    case BehaviorState::ASSIST:
        if (!presence || x < theta_low)
            return BehaviorState::GOODBYE;
        return s;
    case BehaviorState::GOODBYE:
        if (!presence)
            return BehaviorState::IDLE;
        return s;
    default:
        return BehaviorState::IDLE;
    }
}

const char* to_string(BehaviorState s)
{
    switch (s) {
    case BehaviorState::IDLE: return "IDLE";
    case BehaviorState::GREETING: return "GREETING";
    case BehaviorState::SMALLTALK: return "SMALLTALK";
    case BehaviorState::ASSIST: return "ASSIST";
    case BehaviorState::GOODBYE: return "GOODBYE";
    }
    return "UNKNOWN";
}

int main()
{
    BehaviorState s = BehaviorState::IDLE;
    double x = 0.5;
    double theta_low = 0.3, theta_high = 0.8;

    for (int k = 0; k < 10; ++k) {
        bool presence = (k % 2 == 0); // toy example
        s = step_fsm(s, x, presence, theta_low, theta_high);
        std::cout << "k=" << k
                  << " state=" << to_string(s)
                  << " presence=" << presence
                  << " x=" << x
                  << std::endl;
        // here one would update x according to engagement dynamics
    }
    return 0;
}
      
