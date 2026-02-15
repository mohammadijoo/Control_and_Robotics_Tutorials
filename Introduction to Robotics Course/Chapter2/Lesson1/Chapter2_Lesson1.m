
% Piecewise cam profile in MATLAB (usable as a Simulink "MATLAB Function" block)
function x = cam_profile(theta, theta_r, theta_h)
theta = mod(theta, 2*pi);
if theta < theta_r
    a2 = 1/theta_r^2;
    x = a2*theta^2;
elseif theta < theta_h
    x = 1;
else
    b2 = 1/(2*pi-theta_h)^2;
    x = b2*(2*pi-theta)^2;
end
end

% In Simulink:
% 1) Use a Ramp/Clock -> Gain (omega) -> cam_profile(theta)
% 2) Connect output to a Scope to visualize x(t).