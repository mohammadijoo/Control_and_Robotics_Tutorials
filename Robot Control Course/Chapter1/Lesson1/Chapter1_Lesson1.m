
Kp  = 5.0;
tau = 0.2;

Ts     = 1e-3;
T_end  = 1.0;
t      = 0:Ts:T_end;
r      = ones(size(t));
y      = zeros(size(t));
u      = zeros(size(t));

for k = 1:(length(t) - 1)
    e      = r(k) - y(k);
    u(k)   = Kp * e;
    dy     = (-y(k) + u(k)) / tau;
    y(k+1) = y(k) + Ts * dy;
end

plot(t, r, "--", t, y, "LineWidth", 1.5);
xlabel("time (s)");
ylabel("joint position");
legend("reference", "output");
grid on;

% In Simulink, you can instead use a Transfer Fcn block with 1/(tau*s + 1)
% in closed loop with a Gain block (Kp) and a Step input.
