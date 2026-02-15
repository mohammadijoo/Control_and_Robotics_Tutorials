import enum
import math

T_s = 0.1  # sampling time [s]
K_p = 1.0  # proportional gain (must satisfy 0 < T_s * K_p < 2)

class State(enum.Enum):
    IDLE = 0
    NAVIGATE = 1
    CLEAN = 2
    DOCK = 3
    ERROR = 4

class HomeRobotController:
    def __init__(self):
        self.state = State.IDLE
        self.position = 0.0     # 1D position along corridor
        self.goal = 0.0
        self.battery = 1.0      # normalized 0..1
        self.coverage = 0.0     # 0..1

    def set_cleaning_goal(self, goal_position):
        self.goal = goal_position
        if self.state == State.IDLE:
            self.state = State.NAVIGATE

    def low_battery(self):
        return self.battery < 0.2

    def step(self):
        """Single control step at rate 1/T_s."""
        if self.state == State.IDLE:
            # wait for command
            u = 0.0

        elif self.state == State.NAVIGATE:
            e = self.goal - self.position
            u = K_p * e
            # discrete-time position update: x_{k+1} = x_k + T_s * u_k
            self.position += T_s * u
            if abs(e) < 0.05:
                self.state = State.CLEAN
            if self.low_battery():
                self.state = State.DOCK

        elif self.state == State.CLEAN:
            # simple coverage model: increase coverage over time
            u = 0.2  # speed while cleaning
            self.coverage = min(1.0, self.coverage + 0.01)
            if self.coverage >= 0.99:
                self.state = State.DOCK
            if self.low_battery():
                self.state = State.DOCK

        elif self.state == State.DOCK:
            # navigate toward dock at position 0
            e = 0.0 - self.position
            u = K_p * e
            self.position += T_s * u
            if abs(e) < 0.05:
                # instant recharge for this toy model
                self.battery = 1.0
                if self.coverage >= 0.99:
                    self.state = State.IDLE
                else:
                    self.state = State.NAVIGATE

        else:  # ERROR
            u = 0.0

        # battery drains roughly proportional to |u|
        self.battery -= T_s * (0.01 + 0.02 * abs(u))
        self.battery = max(0.0, self.battery)

        return self.state, self.position, self.battery, self.coverage
      
