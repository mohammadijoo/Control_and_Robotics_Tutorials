% Parameters
R  = 10e3;      % 10 kOhm
C  = 1e-6;      % 1 microfarad
RC = R * C;

Rint  = 100e3;
Cint  = 1e-6;
k_int = 1 / (Rint * Cint);

u = @(t) 1.0;   % unit step input

% ODE for RC low-pass
rc_ode = @(t, vC) (u(t) - vC) / RC;

% ODE for op-amp integrator
int_ode = @(t, vo) -k_int * u(t);

tspan = [0 0.1];
vC0   = 0;
vo0   = 0;

[t_rc, vC] = ode45(rc_ode, tspan, vC0);
[t_int, vo] = ode45(int_ode, tspan, vo0);

figure; hold on;
plot(t_rc, vC, "LineWidth", 1.5);
plot(t_int, vo, "LineWidth", 1.5);
xlabel("time (s)");
ylabel("voltage (V)");
legend("RC v_C(t)", "Integrator v_o(t)");
grid on;
title("RC and op-amp integrator dynamics");

% In Simulink, an equivalent RC circuit can be built with:
% - Step block (input u(t))
% - Transfer Fcn block with numerator [1] and denominator [RC 1]
% - Scope block showing v_C(t)
%
% Robotics System Toolbox can then combine this with joint-space models.
