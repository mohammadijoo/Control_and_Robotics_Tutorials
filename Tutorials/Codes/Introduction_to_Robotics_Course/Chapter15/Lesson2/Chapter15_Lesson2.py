from dataclasses import dataclass
from typing import List

@dataclass
class Hazard:
    name: str
    severity: int   # 1..4
    probability: int  # 1..4

def risk_index(h: Hazard) -> int:
    return h.severity * h.probability

def risk_level(I: int) -> str:
    if I <= 4:
        return "low"
    elif I <= 8:
        return "medium"
    else:
        return "high"

hazards: List[Hazard] = [
    Hazard("Pinch at wrist joint", severity=3, probability=3),
    Hazard("Controller overtemperature", severity=2, probability=2),
    Hazard("Unexpected fast motion", severity=4, probability=3),
]

for h in hazards:
    I = risk_index(h)
    print(f"{h.name}: I={I}, level={risk_level(I)}")
      
