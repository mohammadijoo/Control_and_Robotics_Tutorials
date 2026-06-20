import dataclasses
from enum import Enum

class PlannerKind(Enum):
    RRT_STAR = "rrt_star"
    TRAJ_OPT = "traj_opt"

class EstimatorKind(Enum):
    EKF = "ekf"
    PARTICLE = "particle"

@dataclasses.dataclass
class MethodChoice:
    planner: PlannerKind
    estimator: EstimatorKind
    use_learned_policy: bool

@dataclasses.dataclass
class TimingConfig:
    control_dt: float
    plan_dt: float
    perception_dt: float

@dataclasses.dataclass
class ArchitectureConfig:
    methods: MethodChoice
    timing: TimingConfig

class PerceptionModule:
    def __init__(self, config: ArchitectureConfig):
        self.config = config

    def process_sensors(self, raw_msg):
        # Return processed features or intermediate representation
        return {"features": raw_msg.data}

class EstimatorModule:
    def __init__(self, config: ArchitectureConfig):
        self.config = config

    def update_state(self, features, prev_state):
        # Use EKF or particle filter depending on config.methods.estimator
        return prev_state  # placeholder

class PlannerModule:
    def __init__(self, config: ArchitectureConfig):
        self.config = config

    def compute_plan(self, state, goal):
        if self.config.methods.planner is PlannerKind.RRT_STAR:
            # Call OMPL-based planner
            plan = []
        else:
            # Call trajectory optimization method
            plan = []
        return plan

class ControllerModule:
    def __init__(self, config: ArchitectureConfig):
        self.config = config

    def compute_control(self, state, plan):
        # Compute u_t based on current state and plan prefix
        return [0.0, 0.0, 0.0]  # placeholder

class SafetyMonitorModule:
    def __init__(self, config: ArchitectureConfig):
        self.config = config

    def check(self, state, plan, u):
        # Example: clamp velocity or stop if constraints violated
        return u

class AutonomyStack:
    def __init__(self, config: ArchitectureConfig):
        self.config = config
        self.perception = PerceptionModule(config)
        self.estimator = EstimatorModule(config)
        self.planner = PlannerModule(config)
        self.controller = ControllerModule(config)
        self.monitor = SafetyMonitorModule(config)
        self.state = None
        self.plan = None
        self.goal = None

    def step(self, sensor_msg):
        features = self.perception.process_sensors(sensor_msg)
        self.state = self.estimator.update_state(features, self.state)
        if self.plan is None or self._need_replan(self.state, self.plan):
            self.plan = self.planner.compute_plan(self.state, self.goal)
        u = self.controller.compute_control(self.state, self.plan)
        u_safe = self.monitor.check(self.state, self.plan, u)
        return u_safe

    def _need_replan(self, state, plan):
        return False  # placeholder
      
