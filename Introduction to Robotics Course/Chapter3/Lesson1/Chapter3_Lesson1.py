from dataclasses import dataclass
from typing import List, Optional

@dataclass
class Robot:
    name: str
    dof_base: int
    dof_joints: int
    n_agents: int = 1  # >1 indicates swarm
    humanoid: bool = False

    @property
    def dof_total(self) -> int:
        return self.dof_base + self.dof_joints

    def family(self) -> str:
        if self.n_agents > 1:
            return "Swarm"
        if self.humanoid:
            return "Humanoid"
        if self.dof_base > 0 and self.dof_joints == 0:
            return "Mobile robot"
        if self.dof_base == 0 and self.dof_joints > 0:
            return "Manipulator"
        if self.dof_base > 0 and self.dof_joints > 0:
            return "Mobile manipulator"
        return "Other"

robots = [
    Robot("6R arm", dof_base=0, dof_joints=6),
    Robot("Differential drive base", dof_base=3, dof_joints=0),
    Robot("Humanoid example", dof_base=6, dof_joints=22, humanoid=True),
    Robot("Micro-swarm", dof_base=3, dof_joints=0, n_agents=50)
]

for r in robots:
    print(r.name, "=>", r.family(), ", DoF total:", r.dof_total)
      