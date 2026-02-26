
# Joint DoF dictionary and mechanism mobility counter
joint_dof = {
    "R": 1,  # revolute
    "P": 1,  # prismatic
    "H": 1,  # helical
    "C": 2,  # cylindrical
    "U": 2,  # universal
    "S": 3,  # spherical
    "E": 3   # planar
}

def mobility_spatial(n_links, joints):
    """
    n_links: total links including ground
    joints: list like ["R","R","P"]
    """
    f_sum = sum(joint_dof[j] for j in joints)
    j = len(joints)
    return 6*(n_links - 1) - sum((6 - joint_dof[j]) for j in joints)

# Example: 3-link open chain with joints R-R-P
print("Mobility =", mobility_spatial(3, ["R","R","P"]))  # should be 3
      