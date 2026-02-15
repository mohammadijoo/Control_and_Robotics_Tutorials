% Inner and outer classical controllers in a cascade architecture

s = tf('s');

% Fast and slow dynamics
P1 = 1 / (0.05*s + 1);            % fast
P2 = 1 / (s*(0.5*s + 1));         % slower

% Inner PI controller
Kp_in = 5;
Ki_in = 60;
C_in = Kp_in + Ki_in/s;

% Inner closed loop and equivalent plant for the outer loop
T_in  = feedback(C_in*P1, 1);
P_eq  = T_in * P2;

% Outer PI controller
Kp_out = 2;
Ki_out = 1;
C_out  = Kp_out + Ki_out/s;

% Overall two-loop closed loop (r -> y)
T_tot = feedback(C_out*P_eq, 1);

% For comparison: outer loop designed assuming ideal inner unity block
P_ideal = P2;
T_ideal = feedback(C_out*P_ideal, 1);

figure; step(T_tot, T_ideal, 5);
legend('Two-loop actual','Outer loop with ideal inner block');
grid on; title('Role of inner classical loop within complete architecture');
