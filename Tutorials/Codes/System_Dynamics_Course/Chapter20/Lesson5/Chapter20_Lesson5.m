% Chapter20_Lesson5.m
% System Dynamics — Chapter 20, Lesson 5
% Integrated Case Studies and Projects (Mechatronic / Vehicle / Thermal–Fluid)
%
% This MATLAB script:
% 1) Simulates a DC motor with saturation + smooth Coulomb/Stribeck friction.
% 2) Simulates a nonlinear bicycle model under periodic steering.
% 3) Simulates a forced CSTR (mass + energy balance).
% It also extracts Poincaré samples x(kT) for each periodic system.

clear; clc;

%% -----------------------------
% Utilities
%% -----------------------------
poincare = @(t, x, T, nTransient, nPoints) x(:, arrayfun(@(k) find(t >= k*T, 1, 'first'), (nTransient+1):(nTransient+nPoints)));

%% -----------------------------
% Case 1: DC motor
%% -----------------------------
mp.J=2e-3; mp.b=1e-3; mp.Kt=0.05; mp.Ke=0.05; mp.R=1.0; mp.L=5e-3;
mp.tau_c=0.02; mp.tau_s=0.03; mp.w_s=2.0; mp.u_max=12.0;

u_amp=8.0; u_f=0.8;  % u(t)=u_amp*sin(2*pi*u_f*t)
T1=1.25;

motor_ode = @(t,x) motor_f(t,x,mp,u_amp,u_f);

tspan = [0, (50+20)*T1];
x0 = [0; 0; 0]; % [theta; w; i]
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t1,x1] = ode45(motor_ode, tspan, x0, opts);
P1 = poincare(t1, x1', T1, 50, 20);
disp('[Motor] last Poincare sample [theta; w; i]:'), disp(P1(:,end));

%% -----------------------------
% Case 2: Bicycle model
%% -----------------------------
vp.m=1500; vp.Iz=2500; vp.a=1.2; vp.b=1.6; vp.Ux=20;
vp.Cf=80000; vp.Cr=90000; vp.alpha_sat=0.15; vp.delta0=0.06; vp.T=1.0;

bicycle_ode = @(t,x) bicycle_f(t,x,vp);

tspan = [0, (200+50)*vp.T];
x0 = [0;0]; % [vy; r]
[t2,x2] = ode45(bicycle_ode, tspan, x0, opts);
P2 = poincare(t2, x2', vp.T, 200, 50);
disp('[Vehicle] last Poincare sample [vy; r]:'), disp(P2(:,end));

%% -----------------------------
% Case 3: Forced CSTR
%% -----------------------------
cp.V=1.0; cp.rho=1000; cp.Cp=4180; cp.q=1e-3;
cp.CAf=1.0; cp.Tf0=300; cp.dTf=5; cp.T=2.0;
cp.k0=7.2e10; cp.E=8.314e4; cp.Rg=8.314;
cp.dH=-5e7; cp.UA=5e4; cp.Tc=295;

cstr_ode = @(t,x) cstr_f(t,x,cp);

tspan = [0, (400+60)*cp.T];
x0 = [0.9; 305]; % [CA; T]
[t3,x3] = ode45(cstr_ode, tspan, x0, opts);
P3 = poincare(t3, x3', cp.T, 400, 60);
disp('[CSTR] last Poincare sample [CA; T]:'), disp(P3(:,end));

%% -----------------------------
% Local functions
%% -----------------------------
function dx = motor_f(t,x,p,amp,freq)
  th=x(1); w=x(2); i=x(3);
  u = sat(amp*sin(2*pi*freq*t), p.u_max);
  tau_f = p.b*w + stribeck(w,p);
  dth = w;
  dw  = (p.Kt*i - tau_f)/p.J;
  di  = (u - p.R*i - p.Ke*w)/p.L;
  dx = [dth; dw; di];
end

function tau = stribeck(w,p)
  s = tanh(50*w);
  tau = (p.tau_c + (p.tau_s - p.tau_c)*exp(-(abs(w)/p.w_s)^2)) * s;
end

function y = sat(u, umax)
  y = min(max(u, -umax), umax);
end

function F = tire(alpha, C, alpha_sat)
  F = C*alpha_sat*tanh(alpha/alpha_sat);
end

function dx = bicycle_f(t,x,p)
  vy=x(1); r=x(2);
  delta = p.delta0*sin(2*pi*t/p.T);
  alpha_f = (vy + p.a*r)/p.Ux - delta;
  alpha_r = (vy - p.b*r)/p.Ux;
  Fyf = -tire(alpha_f, p.Cf, p.alpha_sat);
  Fyr = -tire(alpha_r, p.Cr, p.alpha_sat);
  dvy = (Fyf + Fyr)/p.m - p.Ux*r;
  dr  = (p.a*Fyf - p.b*Fyr)/p.Iz;
  dx = [dvy; dr];
end

function dx = cstr_f(t,x,p)
  CA=x(1); T=x(2);
  Tf = p.Tf0 + p.dTf*sin(2*pi*t/p.T);
  k = p.k0*exp(-p.E/(p.Rg*T));
  rA = k*CA;
  dCA = (p.q/p.V)*(p.CAf - CA) - rA;
  dT = (p.q/p.V)*(Tf - T) ...
     + (-p.dH/(p.rho*p.Cp))*rA ...
     - (p.UA/(p.rho*p.Cp*p.V))*(T - p.Tc);
  dx = [dCA; dT];
end
