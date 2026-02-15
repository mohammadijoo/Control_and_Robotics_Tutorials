#include <iostream>
#include <cmath>

enum class State { IDLE, NAVIGATE, CLEAN, DOCK, ERROR };

struct HomeRobotController {
    double Ts{0.1};
    double Kp{1.0};
    State state{State::IDLE};
    double position{0.0};
    double goal{0.0};
    double battery{1.0};
    double coverage{0.0};

    bool lowBattery() const { return battery < 0.2; }

    void setCleaningGoal(double goal_pos) {
        goal = goal_pos;
        if (state == State::IDLE) {
            state = State::NAVIGATE;
        }
    }

    void step() {
        double u = 0.0;

        switch (state) {
        case State::IDLE:
            u = 0.0;
            break;

        case State::NAVIGATE: {
            double e = goal - position;
            u = Kp * e;
            position += Ts * u;
            if (std::abs(e) < 0.05) {
                state = State::CLEAN;
            }
            if (lowBattery()) {
                state = State::DOCK;
            }
            break;
        }
        case State::CLEAN:
            u = 0.2;
            coverage = std::min(1.0, coverage + 0.01);
            if (coverage >= 0.99) {
                state = State::DOCK;
            }
            if (lowBattery()) {
                state = State::DOCK;
            }
            break;

        case State::DOCK: {
            double e = 0.0 - position;
            u = Kp * e;
            position += Ts * u;
            if (std::abs(e) < 0.05) {
                battery = 1.0;
                if (coverage >= 0.99) {
                    state = State::IDLE;
                } else {
                    state = State::NAVIGATE;
                }
            }
            break;
        }
        case State::ERROR:
            u = 0.0;
            break;
        }

        battery -= Ts * (0.01 + 0.02 * std::abs(u));
        if (battery < 0.0) battery = 0.0;
    }
};
      
