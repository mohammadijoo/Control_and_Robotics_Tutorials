% config.m (reproducible configuration)
cfg.experiment_name = 'pi_design_second_order';
cfg.plant.num = 1;
cfg.plant.den = [1 2 1];  % (s + 1)^2
cfg.controller.Kp = 4.0;
cfg.controller.Ki = 3.0;
cfg.sim.t_final = 10;
cfg.sim.n_points = 2000;
cfg.tolerances.settling_band = 0.02;

% plant and controller
s = tf('s');
G = tf(cfg.plant.num, cfg.plant.den);
C = cfg.controller.Kp + cfg.controller.Ki / s;

L = C * G;
T = feedback(L, 1);  % unity feedback

% Bode and step plots (saved to files, no manual export)
fig1 = figure('Visible', 'off');
bode(L);
grid on;
title('Open-loop Bode plot');
saveas(fig1, 'matlab_pi_openloop_bode.png');

fig2 = figure('Visible', 'off');
[y, t] = step(T, linspace(0, cfg.sim.t_final, cfg.sim.n_points));
plot(t, y); hold on;
yline(1, '--');
xlabel('Time (s)');
ylabel('Output y(t)');
title('Closed-loop step response (PI)');
grid on;
saveas(fig2, 'matlab_pi_step_response.png');

close(fig1); close(fig2);

% Compute metrics
y_final = y(end);
Mp = max(y) - 1;
ess = 1 - y_final;

idx_last_outside = find(abs(y - 1) > cfg.tolerances.settling_band, 1, 'last');
if isempty(idx_last_outside)
    ts = 0;
else
    ts = t(idx_last_outside);
end

metrics = table(Mp, ess, ts, ...
    'VariableNames', {'Overshoot', 'SteadyStateError', 'SettlingTime'});

writetable(metrics, 'matlab_pi_metrics.csv');

% Example robotics extension:
% If G is derived from a linearized robot joint model via
% linmod('robot_joint_model') or linearize from Robotics System Toolbox,
% the exact same structure applies and metrics remain reproducible.
