def M_3R(q, M0, M1, M2, M3):
    q1, q2, q3 = q
    return (M0
            + np.cos(q2) * M1
            + np.cos(q3) * M2
            + np.cos(q2 + q3) * M3)
      
