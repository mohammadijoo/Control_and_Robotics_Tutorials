
from dataclasses import dataclass
from math import pi, cos, sin

@dataclass
class ArmStructure:
    joint_sequence: str   # e.g., "PPP", "RPP", "RRP", "RRP", "RRPR" etc.
    params: dict          # geometric strokes / lengths

    def dof(self):
        # Each character denotes a 1-DOF joint
        return len(self.joint_sequence)

    def workspace_volume(self):
        s = self.joint_sequence.upper()
        p = self.params

        if s == "PPP":  # Cartesian
            return p["Lx"] * p["Ly"] * p["Lz"]

        if s == "RPP":  # Cylindrical
            rmin, rmax, Lz = p["rmin"], p["rmax"], p["Lz"]
            return pi * (rmax**2 - rmin**2) * Lz

        if s == "RRP":  # Spherical/polar
            rmin, rmax = p["rmin"], p["rmax"]
            phimin, phimax = p["phimin"], p["phimax"]
            return 2*pi*(cos(phimin) - cos(phimax))*(rmax**3 - rmin**3)/3

        if s == "RRP_SCARA":  # tag for SCARA planar R,R plus vertical P
            l1, l2, Lz = p["l1"], p["l2"], p["Lz"]
            A = pi*((l1+l2)**2 - abs(l1-l2)**2)
            return A * Lz

        return None

# Example usage
gantry = ArmStructure("PPP", {"Lx":1.5, "Ly":1.0, "Lz":0.8})
print("Cartesian DOF, volume:", gantry.dof(), gantry.workspace_volume())

scara = ArmStructure("RRP_SCARA", {"l1":0.45, "l2":0.35, "Lz":0.2})
print("SCARA DOF, volume:", scara.dof(), scara.workspace_volume())
      