% Chapter14_Lesson4.m
% System Dynamics (Control Engineering) — Chapter 14, Lesson 4
% Piecewise-Linear Approximations, Saturation, Dead-Zone, and Backlash Models
%
% This script implements the nonlinearities and runs a simple RK4 simulation
% of a 2nd-order plant under a PD controller. Results are written to CSV.
%
% Note: Numerical simulation methods are covered systematically in Chapter 15.
% Here RK4 is used as a practical, self-contained integrator.

clear; clc;

T  = 6.0;
dt = 1e-3;
t  = 0:dt:T;

% PD controller
Kp = 18.0; Kd = 4.5;

% Nonlinearities
u_max = 2.0;      % saturation
d     = 0.25;     % dead-zone half width
b_gap = 0.20;     % backlash width

% Plant parameters
wn = 5.0; zeta = 0.25; b = 1.0;

% Backlash internal state
y_bl = 0.0;

% Optional PWL map approximating tanh(u)
xs = [-3.0 -1.5 -0.5 0.0 0.5 1.5 3.0];
ys = tanh(xs);
use_pwl_map = false;

% Storage
x1 = zeros(size(t));  % position
x2 = zeros(size(t));  % velocity
u_cmd = zeros(size(t));
u_dz  = zeros(size(t));
u_sat = zeros(size(t));
u_bl  = zeros(size(t));

for k = 1:(numel(t)-1)
    r = 1.0; % step reference

    e = r - x1(k);
    u_cmd(k) = Kp*e - Kd*x2(k);

    u_dz(k)  = dead_zone(u_cmd(k), d);
    u_sat(k) = sat(u_dz(k), u_max);

    u_map = u_sat(k);
    if use_pwl_map
        u_map = pwl_eval(xs, ys, u_sat(k));
    end

    [u_bl(k), y_bl] = backlash_step(u_map, y_bl, b_gap);

    % RK4 integration for x' = f(x,u)
    x = [x1(k); x2(k)];
    f = @(xx,uu) plant(xx, uu, wn, zeta, b);

    k1 = f(x, u_bl(k));
    k2 = f(x + 0.5*dt*k1, u_bl(k));
    k3 = f(x + 0.5*dt*k2, u_bl(k));
    k4 = f(x + dt*k3, u_bl(k));

    x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x1(k+1) = x_next(1);
    x2(k+1) = x_next(2);
end

% Last control sample (for completeness)
u_cmd(end) = u_cmd(end-1);
u_dz(end)  = u_dz(end-1);
u_sat(end) = u_sat(end-1);
u_bl(end)  = u_bl(end-1);

tbl = table(t(:), x1(:), x2(:), u_cmd(:), u_dz(:), u_sat(:), u_bl(:), ...
    'VariableNames', {'t','x1','x2','u_cmd','u_deadzone','u_sat','u_backlash'});

writetable(tbl, 'Chapter14_Lesson4_output.csv');
disp('Wrote Chapter14_Lesson4_output.csv');

% Optional quick plots (uncomment if desired)
% figure; plot(t, x1); grid on; xlabel('t'); ylabel('x1');
% figure; plot(t, u_cmd, t, u_bl); grid on; xlabel('t'); ylabel('u'); legend('u\_cmd','u\_backlash');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nonlinearity functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function y = sat(u, umax)
    y = min(max(u, -umax), umax);
end

function y = dead_zone(u, d)
    if abs(u) <= d
        y = 0.0;
    else
        y = u - d*sign(u);
    end
end

function [y, y_state] = backlash_step(u, y_state, b_gap)
    half = 0.5*b_gap;
    if u > y_state + half
        y_state = u - half;
    elseif u < y_state - half
        y_state = u + half;
    end
    y = y_state;
end

function y = pwl_eval(xs, ys, x)
    % Flat extrapolation outside range
    if x <= xs(1), y = ys(1); return; end
    if x >= xs(end), y = ys(end); return; end
    % Find interval
    idx = find(xs <= x, 1, 'last');
    x0 = xs(idx); x1 = xs(idx+1);
    y0 = ys(idx); y1 = ys(idx+1);
    t = (x - x0)/(x1 - x0);
    y = (1-t)*y0 + t*y1;
end

function dx = plant(x, u, wn, zeta, b)
    % x = [x1; x2]
    dx = zeros(2,1);
    dx(1) = x(2);
    dx(2) = -2*zeta*wn*x(2) - (wn^2)*x(1) + b*u;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulink note (blocks):
% - Use "Saturation" block for sat()
% - Use "Dead Zone" block for dead_zone()
% - Use "Backlash" block for backlash_step()
% - Use "1-D Lookup Table" for PWL maps (xs, ys)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
