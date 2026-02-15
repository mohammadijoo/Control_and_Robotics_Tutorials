% Plant and sensor dynamics, e.g. linearized robotic joint
s = tf('s');
G = 10 / (s * (s + 2));   % actuator dynamics
H = 0.5;                  % sensor gain

% Closed-loop TF via Mason (single forward path, single loop)
T_mason = G / (1 + G*H);

% Closed-loop TF via built-in feedback function (negative feedback)
T_matlab = feedback(G, H, -1);

% Verify equality
T_simplified = minreal(T_mason - T_matlab)
