function [state, position, battery, coverage] = ...
    home_robot_step(state, position, goal, battery, coverage)

% States encoded as integers for simplicity
IDLE = 0; NAVIGATE = 1; CLEAN = 2; DOCK = 3; ERROR = 4;

Ts = 0.1;
Kp = 1.0;

u = 0.0;

lowBattery = (battery < 0.2);

switch state
    case IDLE
        % wait for external logic to set state to NAVIGATE
        u = 0.0;

    case NAVIGATE
        e = goal - position;
        u = Kp * e;
        position = position + Ts * u;
        if abs(e) < 0.05
            state = CLEAN;
        end
        if lowBattery
            state = DOCK;
        end

    case CLEAN
        u = 0.2;
        coverage = min(1.0, coverage + 0.01);
        if coverage >= 0.99
            state = DOCK;
        end
        if lowBattery
            state = DOCK;
        end

    case DOCK
        e = 0.0 - position;
        u = Kp * e;
        position = position + Ts * u;
        if abs(e) < 0.05
            battery = 1.0;
            if coverage >= 0.99
                state = IDLE;
            else
                state = NAVIGATE;
            end
        end

    case ERROR
        u = 0.0;
end

battery = battery - Ts * (0.01 + 0.02 * abs(u));
battery = max(0.0, battery);
      
