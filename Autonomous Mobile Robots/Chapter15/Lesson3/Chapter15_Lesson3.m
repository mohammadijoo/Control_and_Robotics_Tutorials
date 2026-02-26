% Chapter15_Lesson3.m
% Dynamic Window Approach (DWA) — MATLAB reference implementation + Simulink build script.
%
% Usage:
%   Chapter15_Lesson3();
%
% Notes:
% - Obstacles are point obstacles (Mx2). Replace with costmap distance queries in real AMR stacks.
% - The Simulink section programmatically builds a simple model for unicycle + DWA controller.

function Chapter15_Lesson3()
    cfg = dwa_config();

    s = struct('x',0,'y',0,'theta',0,'v',0,'w',0);
    goal = [8; 6];

    obs = [];
    obs = [obs; [linspace(2,7,15)' 3*ones(15,1)]];
    obs = [obs; [4*ones(12,1) linspace(1,5,12)']];

    path = [s.x, s.y];
    for k = 1:600
        [v, w, traj] = dwa_control(s, goal, obs, cfg); %#ok<ASGLU>
        s = motion_step(s, v, w, cfg.dt);
        path(end+1,:) = [s.x, s.y]; %#ok<AGROW>
        if norm([s.x; s.y] - goal) < 0.3
            break;
        end
    end

    fprintf('Final state: x=%.3f y=%.3f theta=%.3f (steps=%d)\n', s.x, s.y, s.theta, size(path,1));

    figure; hold on; grid on; axis equal;
    plot(path(:,1), path(:,2), 'LineWidth', 1.5);
    scatter(obs(:,1), obs(:,2), 20, 'filled');
    scatter(goal(1), goal(2), 80, 'p', 'filled');
    legend('trajectory','obstacles','goal');
    title('DWA demo (MATLAB)');
end

function cfg = dwa_config()
    cfg.v_min = -0.2; cfg.v_max = 1.0;
    cfg.w_min = -1.5; cfg.w_max = 1.5;

    cfg.a_max = 0.8;
    cfg.a_brake = 1.0;
    cfg.alpha_max = 2.0;

    cfg.dt = 0.1;
    cfg.T  = 2.0;
    cfg.v_res = 0.05;
    cfg.w_res = 0.1;

    cfg.robot_radius = 0.3;

    cfg.w_heading = 0.4;
    cfg.w_clear   = 0.4;
    cfg.w_speed   = 0.2;
end

function s2 = motion_step(s, v, w, dt)
    s2 = s;
    s2.x = s.x + v*cos(s.theta)*dt;
    s2.y = s.y + v*sin(s.theta)*dt;
    s2.theta = wrapToPi(s.theta + w*dt);
    s2.v = v;
    s2.w = w;
end

function [v_best, w_best, traj_best] = dwa_control(s, goal, obs, cfg)
    [vL, vU, wL, wU] = dynamic_window(s, cfg);

    cand = struct('h',{},'c',{},'sp',{},'v',{},'w',{},'traj',{});
    idx = 0;

    for v = vL:cfg.v_res:(vU + 1e-12)
        for w = wL:cfg.w_res:(wU + 1e-12)
            traj = rollout(s, v, w, cfg);
            clear = min_clearance(traj, obs) - cfg.robot_radius;
            if ~admissible(v, clear, cfg)
                continue;
            end
            idx = idx + 1;
            cand(idx).h = heading_score(traj, goal); %#ok<AGROW>
            cand(idx).c = clear;
            cand(idx).sp = speed_score(v, cfg);
            cand(idx).v = v;
            cand(idx).w = w;
            cand(idx).traj = traj;
        end
    end

    if isempty(cand)
        v_best = 0; w_best = 0; traj_best = {s};
        return;
    end

    hs = normalize([cand.h]);
    cs = normalize([cand.c]);
    ss = normalize([cand.sp]);

    Jbest = -inf; best = 1;
    for i = 1:numel(cand)
        J = cfg.w_heading*hs(i) + cfg.w_clear*cs(i) + cfg.w_speed*ss(i);
        if J > Jbest
            Jbest = J; best = i;
        end
    end

    v_best = cand(best).v;
    w_best = cand(best).w;
    traj_best = cand(best).traj;
end

function [vL, vU, wL, wU] = dynamic_window(s, cfg)
    v_low_dyn = s.v - cfg.a_brake * cfg.dt;
    v_high_dyn = s.v + cfg.a_max * cfg.dt;
    w_low_dyn = s.w - cfg.alpha_max * cfg.dt;
    w_high_dyn = s.w + cfg.alpha_max * cfg.dt;

    vL = max(cfg.v_min, v_low_dyn);
    vU = min(cfg.v_max, v_high_dyn);
    wL = max(cfg.w_min, w_low_dyn);
    wU = min(cfg.w_max, w_high_dyn);
end

function traj = rollout(s0, v, w, cfg)
    n = floor(cfg.T / cfg.dt);
    traj = cell(n+1,1);
    traj{1} = s0;
    s = s0;
    for i = 1:n
        s = motion_step(s, v, w, cfg.dt);
        traj{i+1} = s;
    end
end

function dmin = min_clearance(traj, obs)
    pts = zeros(numel(traj),2);
    for i = 1:numel(traj)
        pts(i,:) = [traj{i}.x, traj{i}.y];
    end
    % pairwise distances N x M
    D = sqrt((pts(:,1) - obs(:,1)').^2 + (pts(:,2) - obs(:,2)').^2);
    dmin = min(D(:));
end

function h = heading_score(traj, goal)
    last = traj{end};
    dir = atan2(goal(2) - last.y, goal(1) - last.x);
    err = wrapToPi(dir - last.theta);
    h = cos(err);
end

function sp = speed_score(v, cfg)
    sp = (v - cfg.v_min) / max(1e-9, (cfg.v_max - cfg.v_min));
end

function ok = admissible(v, clear, cfg)
    dstop = (max(0,v)^2) / max(1e-9, 2*cfg.a_brake);
    ok = clear > (dstop + cfg.robot_radius);
end

function y = normalize(x)
    lo = min(x); hi = max(x);
    if abs(hi - lo) < 1e-12
        y = zeros(size(x));
    else
        y = (x - lo) ./ (hi - lo);
    end
end

function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end

% -------------------------------------------------------------------------
% Simulink: Build a minimal DWA closed-loop model programmatically
% -------------------------------------------------------------------------
function Chapter15_Lesson3_build_simulink()
    mdl = 'Chapter15_Lesson3_DWA_Simulink';
    if bdIsLoaded(mdl); close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    % Blocks: Integrator for x,y,theta (unicycle), MATLAB Function for DWA,
    % and a "To Workspace" for logging.
    add_block('simulink/Sources/Constant', [mdl '/Goal'], 'Value','[8;6]');
    add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/DWA_Controller']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_x']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_y']);
    add_block('simulink/Continuous/Integrator', [mdl '/Int_theta']);

    add_block('simulink/Sinks/To Workspace', [mdl '/x_log'], 'VariableName','x_log');
    add_block('simulink/Sinks/To Workspace', [mdl '/y_log'], 'VariableName','y_log');

    % Positioning (rough)
    set_param([mdl '/Goal'], 'Position', [40 40 80 70]);
    set_param([mdl '/DWA_Controller'], 'Position', [140 30 280 90]);
    set_param([mdl '/Int_x'], 'Position', [340 20 370 50]);
    set_param([mdl '/Int_y'], 'Position', [340 70 370 100]);
    set_param([mdl '/Int_theta'], 'Position', [340 120 370 150]);
    set_param([mdl '/x_log'], 'Position', [420 20 480 50]);
    set_param([mdl '/y_log'], 'Position', [420 70 480 100]);

    % Fill MATLAB Function code (simplified DWA: outputs v,w based on x,y,theta)
    mf = Simulink.findBlocks(mdl, 'BlockType', 'MATLABFunction');
    mf = mf{1};
    code = [
        "function [v,w] = f(goal,x,y,theta)" newline ...
        "%#codegen" newline ...
        "% Minimal placeholder: replace with full dwa_control logic if desired." newline ...
        "dx = goal(1) - x; dy = goal(2) - y;" newline ...
        "dir = atan2(dy,dx);" newline ...
        "err = atan2(sin(dir-theta), cos(dir-theta));" newline ...
        "v = min(0.8, sqrt(dx^2+dy^2));" newline ...
        "w = max(-1.0, min(1.0, 2.0*err));" newline ...
        "end" newline
    ];
    set_param([mdl '/DWA_Controller'], 'Script', code);

    % Wiring: Goal to controller, controller to dynamics, states back
    add_line(mdl, 'Goal/1', 'DWA_Controller/1');
    add_line(mdl, 'Int_x/1', 'DWA_Controller/2');
    add_line(mdl, 'Int_y/1', 'DWA_Controller/3');
    add_line(mdl, 'Int_theta/1', 'DWA_Controller/4');

    % Unicycle dynamics: xdot=v*cos(theta), ydot=v*sin(theta), thetadot=w
    % Use Gain blocks and Trigonometric function blocks for a complete model
    % (omitted for brevity). Students can complete this as an exercise.

    save_system(mdl);
    fprintf('Built Simulink model: %s\n', mdl);
end
