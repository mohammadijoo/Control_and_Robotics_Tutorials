R_WB = [0 -1 0;
        1  0 0;
        0  0 1];
t_WB = [1; 2; 0.5];

p_B = [0.3; 0; 0.2];

p_W = R_WB * p_B + t_WB
p_B_recovered = R_WB.' * (p_W - t_WB)
