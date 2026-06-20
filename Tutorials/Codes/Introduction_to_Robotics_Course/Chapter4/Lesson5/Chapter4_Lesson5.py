
import numpy as np

def payload_ok(m_payload, reach, rated_moment):
    g = 9.81
    M_req = m_payload * g * reach
    return M_req <= rated_moment, M_req

# Example: rated moment at this configuration is 120 N·m
ok, Mreq = payload_ok(m_payload=8.0, reach=0.9, rated_moment=120.0)
print("Required moment:", Mreq, "N·m")
print("Feasible?", ok)
      