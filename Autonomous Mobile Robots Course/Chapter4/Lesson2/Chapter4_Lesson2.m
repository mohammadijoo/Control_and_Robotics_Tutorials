% Chapter4_Lesson2.m
% Autonomous Mobile Robots — Chapter 4 Lesson 2
% Slip, Skid, and Terrain Interaction Models
%
% This MATLAB script includes:
%  (1) hard-ground slip model (linear + saturation),
%  (2) a terramechanics-inspired traction surrogate,
%  (3) an OPTIONAL programmatic Simulink model builder for the same 1D dynamics.
%
% Requirements:
%   - MATLAB
%   - Simulink (only for the Simulink build section)

clear; clc;

%% Parameters
m  = 25.0;      % kg
R  = 0.10;      % m
Iw = 0.05;      % kg*m^2
bw = 0.02;      % N*m*s/rad
g  = 9.81;
Fz = 0.25*m*g;

mu  = 0.8;      % hard-ground friction
Ck  = 15000.0;  % longitudinal stiffness
Crr = 0.02;     % rolling resistance coefficient
Frr = Crr*Fz;

%% Simulation settings
Tcmd = 12.0;    % N*m
dt = 1e-3;
tEnd = 4.0;
t = 0:dt:tEnd;

[vx_h, om_h, kap_h, Fx_h] = simulate_1d(t, dt, Tcmd, "hard", m, R, Iw, bw, Fz, mu, Ck, Frr);
[vx_s, om_s, kap_s, Fx_s] = simulate_1d(t, dt, Tcmd, "soft", m, R, Iw, bw, Fz, mu, Ck, Frr);

%% Plots
figure; plot(t, vx_h, t, vx_s); grid on;
xlabel('time [s]'); ylabel('vx [m/s]'); legend('hard','soft');

figure; plot(t, kap_h, t, kap_s); grid on;
xlabel('time [s]'); ylabel('slip ratio kappa [-]'); legend('hard','soft');

figure; plot(t, Fx_h, t, Fx_s); grid on;
xlabel('time [s]'); ylabel('Fx [N]'); legend('hard','soft');

%% OPTIONAL: Build a Simulink model programmatically
% This section creates a Simulink model "Chapter4_Lesson2_Sim.slx" that simulates
% the same 1D dynamics using Integrator blocks and a MATLAB Function block.
%
% Comment out if you do not have Simulink.
try
    build_simulink_model();
catch ME
    fprintf('Simulink model build skipped or failed: %s\n', ME.message);
end

%% -------- Local functions --------

function kappa = slip_ratio(vx, omega, R)
    eps = 1e-6;
    denom = max([abs(vx), abs(R*omega), eps]);
    kappa = (R*omega - vx)/denom;
end

function Fx = hard_ground_Fx(vx, omega, R, Fz, mu, Ck)
    kappa = slip_ratio(vx, omega, R);
    Fx_lin = Ck*kappa;
    cap = max(mu*Fz, 0);
    Fx = min(max(Fx_lin, -cap), cap);
end

function Fx = soft_soil_Fx(kappa, Fz)
    muPeak = 0.55; kShape = 10.0;
    s = sign(kappa);
    muEff = muPeak*(1 - exp(-kShape*abs(kappa)))*s;
    Fx = Fz*muEff;
end

function [vx, omega, kappa_hist, Fx_hist] = simulate_1d(t, dt, Tcmd, terrain, m, R, Iw, bw, Fz, mu, Ck, Frr)
    N = length(t);
    vx = zeros(1,N);
    omega = zeros(1,N);
    kappa_hist = zeros(1,N);
    Fx_hist = zeros(1,N);

    for i=1:N-1
        k = slip_ratio(vx(i), omega(i), R);
        if terrain == "hard"
            Fx = hard_ground_Fx(vx(i), omega(i), R, Fz, mu, Ck);
        else
            Fx = soft_soil_Fx(k, Fz);
            Fx = Fx - 0.015*Fz*sign(vx(i)); % extra soil motion resistance surrogate
        end

        ax = (Fx - Frr)/m;
        vx(i+1) = vx(i) + dt*ax;

        domega = (Tcmd - R*Fx - bw*omega(i))/Iw;
        omega(i+1) = omega(i) + dt*domega;

        Fx_hist(i) = Fx;
        kappa_hist(i) = k;
    end
    Fx_hist(end) = Fx_hist(end-1);
    kappa_hist(end) = kappa_hist(end-1);
end

function build_simulink_model()
    model = 'Chapter4_Lesson2_Sim';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model); open_system(model);

    % Add blocks
    add_block('simulink/Sources/Constant', [model '/Tcmd'], 'Value', '12');
    add_block('simulink/Continuous/Integrator', [model '/Int_vx'], 'InitialCondition', '0');
    add_block('simulink/Continuous/Integrator', [model '/Int_omega'], 'InitialCondition', '0');

    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/ForcesAndDerivs']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);

    % Parameters as workspace variables (use base workspace)
    assignin('base','m',25.0);
    assignin('base','R',0.10);
    assignin('base','Iw',0.05);
    assignin('base','bw',0.02);
    assignin('base','g',9.81);
    assignin('base','Fz',0.25*25.0*9.81);
    assignin('base','mu',0.8);
    assignin('base','Ck',15000.0);
    assignin('base','Crr',0.02);
    assignin('base','Frr',0.02*(0.25*25.0*9.81));
    assignin('base','terrainFlag',1); % 1=hard, 2=soft

    % Configure MATLAB Function block code
    code = [
        "function [dvx, domega, kappa, Fx] = f(vx, omega, Tcmd)", newline, ...
        "%#codegen", newline, ...
        "eps = 1e-6;", newline, ...
        "den = max([abs(vx), abs(R*omega), eps]);", newline, ...
        "kappa = (R*omega - vx)/den;", newline, ...
        "if terrainFlag == 1", newline, ...
        "  Fx_lin = Ck*kappa;", newline, ...
        "  cap = max(mu*Fz,0);", newline, ...
        "  Fx = min(max(Fx_lin,-cap),cap);", newline, ...
        "else", newline, ...
        "  muPeak = 0.55; kShape = 10.0;", newline, ...
        "  Fx = Fz*(muPeak*(1-exp(-kShape*abs(kappa)))*sign(kappa));", newline, ...
        "  Fx = Fx - 0.015*Fz*sign(vx);", newline, ...
        "end", newline, ...
        "dvx = (Fx - Frr)/m;", newline, ...
        "domega = (Tcmd - R*Fx - bw*omega)/Iw;", newline ...
    ];
    set_param([model '/ForcesAndDerivs'], 'Script', code);

    % Wire blocks
    % Tcmd -> ForcesAndDerivs
    add_line(model, 'Tcmd/1', 'ForcesAndDerivs/3');
    % vx -> ForcesAndDerivs
    add_line(model, 'Int_vx/1', 'ForcesAndDerivs/1');
    % omega -> ForcesAndDerivs
    add_line(model, 'Int_omega/1', 'ForcesAndDerivs/2');

    % dvx -> Int_vx
    add_line(model, 'ForcesAndDerivs/1', 'Int_vx/1');
    % domega -> Int_omega
    add_line(model, 'ForcesAndDerivs/2', 'Int_omega/1');

    % outputs to Scope: vx, omega, kappa, Fx
    add_block('simulink/Signal Routing/Mux', [model '/Mux'], 'Inputs', '4');
    add_line(model, 'Int_vx/1', 'Mux/1');
    add_line(model, 'Int_omega/1', 'Mux/2');
    add_line(model, 'ForcesAndDerivs/3', 'Mux/3');
    add_line(model, 'ForcesAndDerivs/4', 'Mux/4');
    add_line(model, 'Mux/1', 'Scope/1');

    % Layout (rough)
    set_param([model '/Tcmd'], 'Position', [50 80 100 110]);
    set_param([model '/Int_vx'], 'Position', [280 40 310 70]);
    set_param([model '/Int_omega'], 'Position', [280 120 310 150]);
    set_param([model '/ForcesAndDerivs'], 'Position', [160 60 250 140]);
    set_param([model '/Mux'], 'Position', [360 55 390 155]);
    set_param([model '/Scope'], 'Position', [430 80 460 130]);

    % Simulation settings
    set_param(model, 'StopTime', '4');
    save_system(model);
    fprintf('Created Simulink model: %s.slx\n', model);

    % Run once
    sim(model);
end
