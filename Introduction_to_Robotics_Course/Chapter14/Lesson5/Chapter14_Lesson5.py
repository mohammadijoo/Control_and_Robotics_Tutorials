import enum
import math
import random

class BehaviorState(enum.Enum):
    IDLE = 0
    GREETING = 1
    SMALLTALK = 2
    ASSIST = 3
    GOODBYE = 4

class SocialRobot:
    def __init__(self,
                 a=0.9, b=0.3,
                 x_ref=0.7,
                 K=0.8,
                 theta_low=0.3,
                 theta_high=0.8):
        self.a = a
        self.b = b
        self.x = 0.0          # initial engagement
        self.x_ref = x_ref
        self.K = K
        self.theta_low = theta_low
        self.theta_high = theta_high
        self.state = BehaviorState.IDLE

    def engagement_control(self):
        # simple linear feedback u_k = -K (x_k - x_ref)
        return -self.K * (self.x - self.x_ref)

    def update_engagement(self, u, noise_std=0.02):
        w = random.gauss(0.0, noise_std)
        self.x = self.a * self.x + self.b * u + w
        # clip to [0,1]
        self.x = max(0.0, min(1.0, self.x))

    def observe_presence(self):
        # toy model: presence probability increases with engagement
        p = 0.2 + 0.6 * self.x
        return random.random() < p

    def step_behavior_fsm(self, presence):
        x = self.x
        if self.state == BehaviorState.IDLE:
            if presence and x < self.theta_high:
                self.state = BehaviorState.GREETING
        elif self.state == BehaviorState.GREETING:
            # after greeting, move to small talk
            self.state = BehaviorState.SMALLTALK
        elif self.state == BehaviorState.SMALLTALK:
            if x < self.theta_low:
                self.state = BehaviorState.GOODBYE
            elif x > self.theta_high:
                self.state = BehaviorState.ASSIST
        elif self.state == BehaviorState.ASSIST:
            if not presence or x < self.theta_low:
                self.state = BehaviorState.GOODBYE
        elif self.state == BehaviorState.GOODBYE:
            if not presence:
                self.state = BehaviorState.IDLE

    def step(self):
        presence = self.observe_presence()
        self.step_behavior_fsm(presence)
        u = self.engagement_control()
        self.update_engagement(u)
        return self.state, self.x, presence, u

if __name__ == "__main__":
    robot = SocialRobot()
    for k in range(30):
        state, x, presence, u = robot.step()
        print(f"k={k:2d} state={state.name:9s} x={x:5.2f} "
              f"presence={presence} u={u:5.2f}")
      
