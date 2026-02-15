from dataclasses import dataclass

@dataclass
class SpeedSeparationConfig:
    reaction_time: float   # T_r in seconds
    max_decel: float       # a_max > 0 (m/s^2)
    safety_margin: float   # d_safe (m)

def stopping_distance(v: float, cfg: SpeedSeparationConfig) -> float:
    """
    Compute stopping distance given relative speed v.
    v: relative speed (m/s), assumed v >= 0 when approaching.
    """
    if cfg.max_decel <= 0.0:
        raise ValueError("max_decel must be positive")
    return v * cfg.reaction_time + 0.5 * v * v / cfg.max_decel

def safety_function(d: float, v: float, cfg: SpeedSeparationConfig) -> float:
    """
    Safety function h(d, v) = d - d_stop(v) - d_safe.
    Safe if h >= 0, hazardous if h < 0.
    """
    d_stop = stopping_distance(max(v, 0.0), cfg)
    return d - d_stop - cfg.safety_margin

def hazard_active(d: float, v: float, cfg: SpeedSeparationConfig) -> bool:
    h = safety_function(d, v, cfg)
    return h < 0.0

def emergency_stop():
    # In a real robot, this would send a message to a safety PLC or low-level controller.
    print("!!! EMERGENCY STOP !!!")

def monitor_step(d: float, v: float, cfg: SpeedSeparationConfig):
    """
    One monitoring step; call this in a periodic loop.
    """
    if hazard_active(d, v, cfg):
        emergency_stop()
      
