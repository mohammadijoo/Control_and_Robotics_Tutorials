% Chapter14_Lesson4.m
% Recovery Behaviors and Fault Handling — MATLAB + Simulink-friendly supervisor (teaching example)
%
% This script:
% 1) Simulates a navigation "progress" signal d(t) and cmd signals v(t), w(t)
% 2) Implements a discrete-time fault monitor (no-progress + oscillation)
% 3) Demonstrates a recovery escalation policy
% 4) (Optional) Creates a simple Simulink model programmatically with a MATLAB Function block
%
% NOTE: For full architecture, you typically implement the supervisor in Stateflow.

clear; clc;

%% Parameters
dt = 0.1;                 % sample time [s]
T  = 40.0;                % total time [s]
N  = round(T/dt);

window_s = 10.0;
min_progress_m = 0.25;

osc_window_s = 6.0;
osc_sign_flips = 6;
v_eps = 0.03;

max_attempts = 3;

spin_time_s = 3.0;
backup_time_s = 2.0;

%% Toy signals (stagnation + oscillatory commands)
t = (0:N-1)*dt;

% distance-to-goal: decreases then stalls
d = 5.0 - 0.08*t;
stall_idx = t > 15;
d(stall_idx) = d(find(~stall_idx,1,'last')) - 0.005*(t(stall_idx)-t(find(~stall_idx,1,'last')));

% cmd signals: start OK, then oscillate after 15s
v = 0.2*ones(size(t));
w = zeros(size(t));
osc_idx = t > 15;
v(osc_idx) = 0.12 * sign(sin(2*pi*0.8*(t(osc_idx)-15)));  % flip sign ~1.6 Hz
w(osc_idx) = 0.0;

%% Supervisor state
attempts = 0;
state = "NAVIGATE";  % NAVIGATE, RECOVER_SPIN, RECOVER_BACKUP, SAFE_STOP
state_timer = 0.0;

hist_d = [];  % columns: [t, d]
hist_cmd = []; % columns: [t, v, w]

log_state = strings(N,1);

%% Helpers
trim_hist = @(H, win) H(H(:,1) >= (H(end,1)-win), :);

sign_eps = @(x) (abs(x) < v_eps).*0 + (x >= v_eps).*1 + (x <= -v_eps).*(-1);

progress_ok = @(Hd) (size(Hd,1) < 2) || ((Hd(1,2) - Hd(end,2)) >= min_progress_m);

oscillating = @(Hc) ( ...
    size(Hc,1) >= 3 && ...
    ( ...
      sum( (sign_eps(Hc(2:end,2)) ~= 0) & (sign_eps(Hc(1:end-1,2)) ~= 0) & ...
           (sign_eps(Hc(2:end,2)) ~= sign_eps(Hc(1:end-1,2))) ) + ...
      sum( (sign_eps(Hc(2:end,3)) ~= 0) & (sign_eps(Hc(1:end-1,3)) ~= 0) & ...
           (sign_eps(Hc(2:end,3)) ~= sign_eps(Hc(1:end-1,3))) ) ...
    ) >= osc_sign_flips ...
);

%% Main loop
for k = 1:N
    % update histories
    hist_d = [hist_d; t(k), d(k)];
    hist_cmd = [hist_cmd; t(k), v(k), w(k)];
    hist_d = trim_hist(hist_d, window_s);
    hist_cmd = trim_hist(hist_cmd, osc_window_s);

    no_progress = ~progress_ok(hist_d);
    osc = oscillating(hist_cmd);

    % State machine
    switch state
        case "NAVIGATE"
            if attempts >= max_attempts
                state = "SAFE_STOP";
                state_timer = 0.0;
            elseif no_progress || osc
                state = "RECOVER_SPIN";
                state_timer = 0.0;
                attempts = attempts + 1;
            end

        case "RECOVER_SPIN"
            state_timer = state_timer + dt;
            if state_timer >= spin_time_s
                state = "RECOVER_BACKUP";
                state_timer = 0.0;
            end

        case "RECOVER_BACKUP"
            state_timer = state_timer + dt;
            if state_timer >= backup_time_s
                state = "NAVIGATE";
                state_timer = 0.0;
                % after recovery, clear histories to avoid immediate re-trigger
                hist_d = hist_d(end,:);
                hist_cmd = hist_cmd(end,:);
            end

        case "SAFE_STOP"
            % do nothing; remain stopped
            state_timer = state_timer + dt;
    end

    log_state(k) = state;
end

%% Plot
figure; 
subplot(3,1,1); plot(t, d); grid on; ylabel('d(t) [m]'); title('Progress signal');
subplot(3,1,2); plot(t, v); grid on; ylabel('v(t) [m/s]');
subplot(3,1,3); plot(t, double(categorical(log_state))); grid on; ylabel('state'); xlabel('t [s]');

disp("Final state: " + log_state(end) + ", recovery attempts=" + attempts);

%% Optional: create a minimal Simulink model with a MATLAB Function block
% This is intentionally minimal; for production use, use Stateflow.
try
    mdl = "Chapter14_Lesson4_Sim";
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block("simulink/Sources/From Workspace", mdl + "/d_in", "VariableName", "d");
    add_block("simulink/Sources/From Workspace", mdl + "/v_in", "VariableName", "v");
    add_block("simulink/Sources/From Workspace", mdl + "/w_in", "VariableName", "w");
    add_block("simulink/User-Defined Functions/MATLAB Function", mdl + "/Supervisor");

    add_block("simulink/Sinks/To Workspace", mdl + "/state_out", "VariableName", "state_out");

    set_param(mdl + "/Supervisor", "Script", [
        "function st = fcn(d,v,w)\n" ...
        "% Simple placeholder: output 0=NAV,1=RECOVER,2=STOP\n" ...
        "persistent cnt\n" ...
        "if isempty(cnt), cnt=0; end\n" ...
        "if abs(v) < 0.01\n" ...
        "  cnt = cnt + 1;\n" ...
        "else\n" ...
        "  cnt = 0;\n" ...
        "end\n" ...
        "if cnt > 30\n" ...
        "  st = 1;\n" ...
        "else\n" ...
        "  st = 0;\n" ...
        "end\n" ...
        "end\n"
    ]);

    add_line(mdl, "d_in/1", "Supervisor/1");
    add_line(mdl, "v_in/1", "Supervisor/2");
    add_line(mdl, "w_in/1", "Supervisor/3");
    add_line(mdl, "Supervisor/1", "state_out/1");

    set_param(mdl, "StopTime", num2str(T));
    save_system(mdl);
    disp("Created Simulink model: " + mdl + " (saved).");
catch ME
    disp("Simulink model creation skipped (Simulink may be unavailable):");
    disp(ME.message);
end
