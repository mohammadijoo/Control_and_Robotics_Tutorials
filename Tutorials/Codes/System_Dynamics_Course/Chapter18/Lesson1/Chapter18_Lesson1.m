% Chapter18_Lesson1.m
% Work, Energy, and Power in Mechanical, Electrical, Fluid, and Thermal Systems
% MATLAB script (also suitable for translation to Simulink subsystems)

clear; clc;

t0 = 0; tf = 12; dt = 1e-4;
t = t0:dt:tf;
N = numel(t);

trapz_local = @(y, tt) trapz(tt, y);

%% 1) Mechanical: m*xdd + c*xd + k*x = F(t)
m = 1.5; c = 0.8; ks = 12.0;
Fin = @(tt) 2.5*sin(1.2*tt) + 1.2*cos(0.4*tt);

x = zeros(1,N); v = zeros(1,N);
Pm_in = zeros(1,N); Pm_diss = zeros(1,N); Em = zeros(1,N);

for k = 1:N-1
    f = Fin(t(k));
    a = (f - c*v(k) - ks*x(k))/m;
    v(k+1) = v(k) + dt*a;
    x(k+1) = x(k) + dt*v(k);

    Pm_in(k) = f*v(k);
    Pm_diss(k) = c*v(k)^2;
    Em(k) = 0.5*m*v(k)^2 + 0.5*ks*x(k)^2;
end
Pm_in(N) = Fin(t(N))*v(N);
Pm_diss(N) = c*v(N)^2;
Em(N) = 0.5*m*v(N)^2 + 0.5*ks*x(N)^2;
mechResidual = Em(end) - Em(1) - (trapz_local(Pm_in,t) - trapz_local(Pm_diss,t));

%% 2) Electrical: series RLC
R = 2.0; L = 0.6; C = 0.25;
Vin = @(tt) 5.0*sin(2.0*tt) + 2.0*cos(0.5*tt);

q = zeros(1,N); i = zeros(1,N);
Pe_in = zeros(1,N); Pe_diss = zeros(1,N); Ee = zeros(1,N);

for k = 1:N-1
    vin = Vin(t(k));
    di = (vin - R*i(k) - q(k)/C)/L;
    dq = i(k);

    i(k+1) = i(k) + dt*di;
    q(k+1) = q(k) + dt*dq;

    Pe_in(k) = vin*i(k);
    Pe_diss(k) = R*i(k)^2;
    Ee(k) = 0.5*L*i(k)^2 + 0.5*q(k)^2/C;
end
Pe_in(N) = Vin(t(N))*i(N);
Pe_diss(N) = R*i(N)^2;
Ee(N) = 0.5*L*i(N)^2 + 0.5*q(N)^2/C;
elecResidual = Ee(end) - Ee(1) - (trapz_local(Pe_in,t) - trapz_local(Pe_diss,t));

%% 3) Fluid (small-signal hydraulic chamber + line)
Ch = 0.08; Lh = 0.15; Rh = 1.7;
QinHyd = @(tt) 0.8*sin(0.9*tt) + 0.2*cos(0.3*tt);

p = zeros(1,N); qf = zeros(1,N);
Pf_in = zeros(1,N); Pf_diss = zeros(1,N); Ef = zeros(1,N);

for k = 1:N-1
    qsrc = QinHyd(t(k));
    dp = (qsrc - qf(k))/Ch;
    dqf = (p(k) - Rh*qf(k))/Lh;

    p(k+1) = p(k) + dt*dp;
    qf(k+1) = qf(k) + dt*dqf;

    Pf_in(k) = p(k)*qsrc;
    Pf_diss(k) = Rh*qf(k)^2;
    Ef(k) = 0.5*Ch*p(k)^2 + 0.5*Lh*qf(k)^2;
end
Pf_in(N) = p(N)*QinHyd(t(N));
Pf_diss(N) = Rh*qf(N)^2;
Ef(N) = 0.5*Ch*p(N)^2 + 0.5*Lh*qf(N)^2;
fluidResidual = Ef(end) - Ef(1) - (trapz_local(Pf_in,t) - trapz_local(Pf_diss,t));

%% 4) Thermal: Cth*dT/dt = Qdot_in - (T - Tenv)/Rth
Cth = 600; Rth = 0.4; Tenv = 293.15;
QdotTherm = @(tt) 120 + 40*sin(0.15*tt);

T = Tenv*ones(1,N);
Pt_in = zeros(1,N); Pt_out = zeros(1,N); Et = zeros(1,N);

for k = 1:N-1
    qdot_in = QdotTherm(t(k));
    qdot_out = (T(k) - Tenv)/Rth;
    dT = (qdot_in - qdot_out)/Cth;

    T(k+1) = T(k) + dt*dT;

    Pt_in(k) = qdot_in;
    Pt_out(k) = qdot_out;
    Et(k) = Cth*(T(k) - Tenv);
end
Pt_in(N) = QdotTherm(t(N));
Pt_out(N) = (T(N) - Tenv)/Rth;
Et(N) = Cth*(T(N) - Tenv);
thermResidual = Et(end) - Et(1) - (trapz_local(Pt_in,t) - trapz_local(Pt_out,t));

%% Report
fprintf('=== Energy Balance Audit (MATLAB) ===\n');
fprintf('Mechanical residual : %.6e\n', mechResidual);
fprintf('Electrical residual : %.6e\n', elecResidual);
fprintf('Fluid residual      : %.6e\n', fluidResidual);
fprintf('Thermal residual    : %.6e\n\n', thermResidual);

fprintf('Final stored energies:\n');
fprintf('E_mech(tf)  = %.6f\n', Em(end));
fprintf('E_elec(tf)  = %.6f\n', Ee(end));
fprintf('E_fluid(tf) = %.6f\n', Ef(end));
fprintf('E_therm(tf) = %.6f\n', Et(end));

%% Simulink note (manual build)
% Mechanical subsystem: Integrator(a->v), Integrator(v->x), Sum, Gain blocks
% Electrical subsystem: Integrator(di->i), Integrator(i->q), Sum, Gain blocks
% Fluid subsystem: Integrator(dp->p), Integrator(dq->qf), Sum, Gain blocks
% Thermal subsystem: Integrator(dT->T), Sum, Gain blocks
% Add Product blocks for power and Integrator blocks for work.
