% Chapter14_Lesson3.m
% Autonomous Mobile Robots (AMR) — Chapter 14 Lesson 3
% Behavior Trees / State Machines for Navigation
%
% This file provides:
% 1) A hierarchical finite-state machine (HFSM) navigation controller in MATLAB
% 2) (Optional) Programmatic creation of a simple Simulink model shell and
%    a placeholder Stateflow chart (if Stateflow is available on your installation).
%
% Note: The BT implementation is demonstrated in other languages; MATLAB here
% focuses on HFSM/Stateflow patterns that are common for embedded navigation logic.

function Chapter14_Lesson3()
    % Blackboard-like struct
    bb.have_goal = true;
    bb.dist_to_goal = 5.0;
    bb.goal_tol = 0.2;
    bb.P_trace = 0.8;
    bb.P_trace_max = 2.0;
    bb.progress_per_tick = 0.2;
    bb.obstacle_blocking = false;
    bb.path_valid = false;
    bb.recoveries = 0;
    bb.planner_calls = 0;

    fsm.state = NavState.IDLE;
    fsm.stall = 0;
    fsm.stall_max = 10;

    fprintf('=== HFSM demo (MATLAB) ===\n');
    for t = 0:59
        if t == 12
            bb.obstacle_blocking = true;
        end
        if t == 25
            bb.obstacle_blocking = false;
        end

        [fsm, bb] = fsm_step(fsm, bb);

        fprintf('t=%02d state=%-9s dist=%.2f path_valid=%d rec=%d\n', ...
            t, char(fsm.state), bb.dist_to_goal, bb.path_valid, bb.recoveries);

        if fsm.state == NavState.DONE
            break;
        end
        pause(0.02);
    end

    % Optional: generate a Simulink/Stateflow skeleton
    % Uncomment the line below if you want to auto-create a model.
    % build_simulink_stateflow_skeleton();
end

% -----------------------------
% State definitions
% -----------------------------
classdef NavState
   enumeration
      IDLE, PLAN, CONTROL, RECOVERY, DONE, FAIL
   end
end

% -----------------------------
% Guard conditions
% -----------------------------
function ok = have_goal(bb)
    ok = isfield(bb,'have_goal') && bb.have_goal;
end

function ok = localization_ok(bb)
    ok = bb.P_trace <= bb.P_trace_max;
end

function ok = path_valid(bb)
    ok = bb.path_valid;
end

function ok = goal_reached(bb)
    ok = bb.dist_to_goal <= bb.goal_tol;
end

% -----------------------------
% Actions
% -----------------------------
function [bb, out] = act_global_plan(bb)
    if ~have_goal(bb) || ~localization_ok(bb)
        out = "failure"; return;
    end
    bb.path_valid = true;
    bb.planner_calls = bb.planner_calls + 1;
    out = "success";
end

function [bb, out] = act_local_control(bb)
    if ~path_valid(bb)
        out = "failure"; return;
    end
    if bb.obstacle_blocking
        out = "running"; return;
    end
    bb.dist_to_goal = max(0.0, bb.dist_to_goal - bb.progress_per_tick);
    out = "running";
    if goal_reached(bb), out = "success"; end
end

function [bb, out] = act_recovery_clear_costmap(bb)
    bb.obstacle_blocking = false;
    bb.path_valid = false;
    bb.recoveries = bb.recoveries + 1;
    out = "success";
end

% -----------------------------
% HFSM step
% -----------------------------
function [fsm, bb] = fsm_step(fsm, bb)
    switch fsm.state
        case NavState.IDLE
            if have_goal(bb) && localization_ok(bb)
                fsm.state = NavState.PLAN;
            else
                fsm.state = NavState.FAIL;
            end

        case NavState.PLAN
            [bb, out] = act_global_plan(bb);
            if out == "success"
                fsm.state = NavState.CONTROL;
            else
                fsm.state = NavState.FAIL;
            end

        case NavState.CONTROL
            if goal_reached(bb)
                fsm.state = NavState.DONE;
                return;
            end
            [bb, out] = act_local_control(bb);
            if out == "failure"
                fsm.state = NavState.PLAN;
                return;
            end
            if bb.obstacle_blocking
                fsm.stall = fsm.stall + 1;
                if fsm.stall >= fsm.stall_max
                    fsm.stall = 0;
                    fsm.state = NavState.RECOVERY;
                end
            else
                fsm.stall = 0;
            end

        case NavState.RECOVERY
            [bb, ~] = act_recovery_clear_costmap(bb);
            fsm.state = NavState.PLAN;

        otherwise
            % DONE or FAIL -> no-op
    end
end

% -----------------------------
% Optional Simulink/Stateflow skeleton builder
% -----------------------------
function build_simulink_stateflow_skeleton()
    % This creates a minimal Simulink model and attempts to add a Stateflow chart.
    % If Stateflow isn't licensed/installed, this will error; keep it optional.
    mdl = 'Chapter14_Lesson3_Simulink';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    if exist([mdl '.slx'],'file')
        delete([mdl '.slx']);
    end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Sources/Constant', [mdl '/have_goal'], 'Value', '1');
    add_block('simulink/Sources/Constant', [mdl '/P_trace'], 'Value', '0.8');
    add_block('simulink/Sources/Constant', [mdl '/P_trace_max'], 'Value', '2.0');
    add_block('simulink/Sources/Constant', [mdl '/dist_to_goal'], 'Value', '5.0');
    add_block('simulink/Sources/Constant', [mdl '/goal_tol'], 'Value', '0.2');

    % MATLAB Function placeholder (for HFSM logic)
    blk = add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/HFSM_Controller']);
    set_param(blk, 'Position', [300 80 500 180]);

    % Try adding Stateflow chart if available
    try
        rt = sfroot;
        m = rt.find('-isa','Simulink.BlockDiagram','Name',mdl);
        ch = Stateflow.Chart(m);
        ch.Name = 'NavLogic';
        % A real implementation would add states/transitions here.
        disp('Stateflow chart created: NavLogic (populate states/transitions manually).');
    catch ME
        warning('Stateflow not available or error occurred: %s', ME.message);
    end

    save_system(mdl);
    disp(['Saved model: ' mdl '.slx']);
end
