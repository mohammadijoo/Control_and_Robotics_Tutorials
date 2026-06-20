\
% Chapter 3 — Lesson 4: Motion Primitives for Ground Vehicles (conceptual use)

clear; clc;
v = 0.6; kappa = 0.4; T = 1.2;
dtheta = v*kappa*T;
if abs(kappa) < 1e-12
    dx = v*T; dy = 0;
else
    dx = sin(dtheta)/kappa;
    dy = (1 - cos(dtheta))/kappa;
end
fprintf('dx=%f dy=%f dtheta=%f\n', dx, dy, dtheta);
