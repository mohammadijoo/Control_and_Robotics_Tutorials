class ReactiveWallAvoider:
    """
    Simple reactive controller:
      - If front distance d_front < d_safe: turn in place.
      - Else: drive forward.
    """
    def __init__(self, d_safe=0.7):
        self.d_safe = d_safe

    def compute_action(self, obs):
        """
        obs: dictionary with key "d_front" (meters).
        Returns a dict with linear and angular velocity.
        """
        d_front = obs["d_front"]
        if d_front < self.d_safe:
            # obstacle too close: stop and turn left
            return {"v": 0.0, "w": 1.0}
        else:
            # path clear: move forward
            return {"v": 0.4, "w": 0.0}


# Example usage
controller = ReactiveWallAvoider(d_safe=0.5)
obs = {"d_front": 0.3}
u = controller.compute_action(obs)
print("Command:", u)  # Command: {'v': 0.0, 'w': 1.0}
      
