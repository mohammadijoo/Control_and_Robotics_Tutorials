% Chapter2_Lesson4.m
% Autonomous Mobile Robots — Chapter 2 Lesson 4
% Omnidirectional Bases (mecanum, Swedish wheels)
%
% This script demonstrates:
%   1) 4-wheel mecanum inverse/forward kinematics
%   2) A programmatically-generated Simulink model that implements w = J*v
%
% Toolbox notes:
%   - Basic MATLAB: matrix ops, pinv
%   - Simulink: model creation (new_system, add_block, etc.)
%   - Robotics System Toolbox (optional): for ROS Twist message interfaces

function Chapter2_Lesson4()
    % Geometry
    r = 0.05;     % wheel radius [m]
    lx = 0.20;    % half-length in x [m]
    ly = 0.15;    % half-width  in y [m]
    alpha = deg2rad(45); % skew angle [rad]
    a = lx + ly;
    kappa = r*cos(alpha);

    % Jacobian: w = (1/kappa) * A * v, v = [vx; vy; omega]
    J = (1/kappa) * [ 1  -1  -a;
                      1   1   a;
                      1   1  -a;
                      1  -1   a ];

    % Example command
    v = [0.40; -0.10; 0.60];
    w = J*v;

    % Forward (least squares)
    v_hat = pinv(J)*w;

    disp('Commanded v = [vx; vy; omega]'); disp(v);
    disp('Wheel speeds w = [w1; w2; w3; w4]'); disp(w);
    disp('Reconstructed v_hat (LS)'); disp(v_hat);

    % Build a simple Simulink model implementing w = J*v
    build_simulink_model(J);
    fprintf('Simulink model generated: Chapter2_Lesson4_Simulink.slx (in current folder)\n');
end

function build_simulink_model(J)
    mdl = 'Chapter2_Lesson4_Simulink';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    if exist([mdl '.slx'], 'file')
        delete([mdl '.slx']);
    end

    new_system(mdl);
    open_system(mdl);

    % Blocks: v (3x1) constant -> Gain (J) -> w (4x1) -> Scope
    add_block('simulink/Sources/Constant', [mdl '/v_cmd'], 'Value', 'v_cmd');
    add_block('simulink/Math Operations/Gain', [mdl '/J_gain'], 'Gain', 'J');
    add_block('simulink/Sinks/Scope', [mdl '/w_scope']);

    % Set positions
    set_param([mdl '/v_cmd'], 'Position', [80 70 140 110]);
    set_param([mdl '/J_gain'], 'Position', [220 60 320 120]);
    set_param([mdl '/w_scope'], 'Position', [420 65 450 115]);

    % Wire
    add_line(mdl, 'v_cmd/1', 'J_gain/1');
    add_line(mdl, 'J_gain/1', 'w_scope/1');

    % Workspace variables used by the model
    assignin('base', 'J', J);
    assignin('base', 'v_cmd', [0.40; -0.10; 0.60]);

    save_system(mdl);
end
