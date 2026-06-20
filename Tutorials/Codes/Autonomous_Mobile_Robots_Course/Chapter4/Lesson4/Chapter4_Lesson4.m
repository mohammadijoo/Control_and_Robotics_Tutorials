% Chapter4_Lesson4.m
% Autonomous Mobile Robots (Control Engineering)
% Chapter 4: Mobile Robot Dynamics (Applied)
% Lesson 4: Stability and Tip-Over Risk (vehicle view)
%
% MATLAB script:
% 1) Computes quasi-static LTR-based tip-over risk along a synthetic trajectory
% 2) Builds a simple roll compliance model in Simulink programmatically (optional)
%
% Toolboxes (optional, not required):
% - Robotics System Toolbox (for integration in AMR stacks)
% - Simulink (for roll model)

clear; clc;

params.track     = 0.55;   % [m]
params.wheelbase = 0.65;   % [m]
params.h         = 0.25;   % [m] cg height
params.mu        = 0.70;   % [-]
params.g         = 9.81;   % [m/s^2]

a_tip_y = params.g * (params.track / (2*params.h));
a_mu    = params.mu * params.g;

fprintf("a_tip_y = %.3f m/s^2\n", a_tip_y);
fprintf("mu*g    = %.3f m/s^2\n\n", a_mu);

% Trajectory (t, v, kappa)
t = linspace(0, 12, 601)';
v = 0.2 + 1.4 * (1 - exp(-t/2.5)) .* (1 - 0.4 * (t>8) .* (t-8)/4);
v = max(0, min(1.8, v));
kappa = 0.05 + 0.35 * exp(-((t-6)/2).^2);

% Accelerations
a_y = (v.^2) .* kappa;
a_x = [diff(v)./diff(t); 0];
a_x(end) = a_x(end-1);

% LTRs
LTR_y = 2*params.h .* a_y ./ (params.g * params.track);
LTR_x = 2*params.h .* a_x ./ (params.g * params.wheelbase);

margin_tip_y = 1 - abs(LTR_y);
margin_tip_x = 1 - abs(LTR_x);

a_planar = sqrt(a_x.^2 + a_y.^2);
margin_slide = a_mu - a_planar;

fprintf("Min margin_tip_y = %.3f\n", min(margin_tip_y));
fprintf("Min margin_slide = %.3f\n\n", min(margin_slide));

% Conservative safe speed bounds along curvature
k = max(abs(kappa), 1e-9);
v_tip   = sqrt(a_tip_y ./ k);
v_slide = sqrt(a_mu ./ k);
v_safe  = min(v_tip, v_slide);

figure; plot(t, v, t, v_safe);
xlabel("t [s]"); ylabel("speed [m/s]");
legend("v(t)", "v_safe(t)"); grid on;

figure; plot(t, LTR_y); hold on;
plot(t, sign(LTR_y), '--');
xlabel("t [s]"); ylabel("LTR_y"); legend("LTR_y", "tip threshold"); grid on;

figure; plot(t, margin_tip_y); hold on;
plot(t, margin_slide);
yline(0,'--');
xlabel("t [s]"); ylabel("margin"); legend("margin_tip_y", "margin_slide"); grid on;

% Optional: programmatically build a small roll model in Simulink
% (This creates Chapter4_Lesson4_RollModel.slx in the current folder.)
try
    buildRollModel(params);
catch ME
    fprintf("Simulink model build skipped: %s\n", ME.message);
end

function buildRollModel(params)
    mdl = "Chapter4_Lesson4_RollModel";
    if bdIsLoaded(mdl), close_system(mdl,0); end
    if exist(mdl + ".slx","file"), delete(mdl + ".slx"); end

    new_system(mdl); open_system(mdl);

    % Roll dynamics (small-angle):
    % I_phi * phi_ddot + c_phi * phi_dot + k_phi * phi = m*h*a_y
    % We implement transfer function: phi(s)/a_y(s) = (m*h)/(I*s^2 + c*s + k)
    %
    % Choose nominal parameters (student can tune)
    m  = 35;        % [kg]
    h  = params.h;  % [m]
    I  = 6.0;       % [kg*m^2]
    c  = 10.0;      % [N*m*s/rad]
    k  = 250.0;     % [N*m/rad]

    add_block("simulink/Sources/In1", mdl + "/a_y");
    add_block("simulink/Continuous/Transfer Fcn", mdl + "/RollTF");
    add_block("simulink/Sinks/Out1", mdl + "/phi");

    set_param(mdl + "/RollTF", "Numerator", mat2str([m*h]), ...
                             "Denominator", mat2str([I c k]));

    add_line(mdl, "a_y/1", "RollTF/1");
    add_line(mdl, "RollTF/1", "phi/1");

    set_param(mdl, "StopTime", "10");
    save_system(mdl);
    fprintf("Created %s.slx\n", mdl);
end
