
function [dx, y] = admittance1d_ct(t, x, Fext, M_a, D_a, K_a, x0)
% x(1) = x_r, x(2) = xdot_r
xr = x(1);
xrdot = x(2);

xddot = (Fext - D_a * xrdot - K_a * (xr - x0)) / M_a;

dx = zeros(2,1);
dx(1) = xrdot;
dx(2) = xddot;

y = xr;
end
