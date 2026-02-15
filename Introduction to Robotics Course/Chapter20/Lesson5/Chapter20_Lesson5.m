% Load CSV: columns t, r, y, u
data = readmatrix("log.csv");
t = data(:, 1);
r = data(:, 2);
y = data(:, 3);
u = data(:, 4);

dt = mean(diff(t));
e = r - y;

J_ISE = sum(e.^2) * dt;
e_RMS = sqrt(mean(e.^2));

fprintf("ISE = %.3f\n", J_ISE);
fprintf("RMS error = %.3f\n", e_RMS);

% Plot for the report / slides
figure;
plot(t, r, "LineWidth", 1.2); hold on;
plot(t, y, "LineWidth", 1.2);
xlabel("time [s]");
ylabel("position");
legend("reference", "output");
grid on;

% Simulink integration:
%  - Place 'To Workspace' blocks on outputs r, y, u.
%  - Run: sim("capstone_model");
%  - Then use simout_y.time and simout_y.signals.values
      
