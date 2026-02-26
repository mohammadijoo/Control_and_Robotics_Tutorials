
import numpy as np

def classify_system(c_T, i_EU, r_R,
                    th_c=0.3, th_i=0.2, th_r=2):
    """
    c_T: task capacity proxy in [0,1]
    i_EU: normalized environment-action coupling proxy in [0,1]
    r_R: reprogrammability rank proxy (integer >=1)
    """
    if c_T < th_c and i_EU < th_i and r_R == 1:
        return "Automation"
    if c_T >= th_c and i_EU >= th_i and r_R >= th_r:
        return "Robot"
    return "Mechatronic System"

examples = [
    ("CNC line",        0.1, 0.05, 1),
    ("Active suspension",0.4, 0.15, 2),
    ("Mobile robot",    0.7, 0.6, 5),
]

for name, cT, iEU, rR in examples:
    print(name, "->", classify_system(cT, iEU, rR))
      