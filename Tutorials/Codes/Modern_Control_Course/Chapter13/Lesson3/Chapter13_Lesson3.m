% Chapter13_Lesson3.m
% Detectability and Stable Unobservable Modes
%
% This script uses the unobservable-subspace viewpoint.
% Continuous time: all unobservable modes satisfy real(lambda) < 0.
% Discrete time: all unobservable modes satisfy abs(lambda) < 1.
%
% Optional Control System Toolbox functions such as obsv are not required.

clear; clc;

A1 = diag([-1, 2, -0.5]);
C1 = [0 1 0];
reportDetectability(A1, C1, "Continuous-time: detectable but not observable", "continuous");

A2 = diag([1, -2, -0.5]);
C2 = [0 1 0];
reportDetectability(A2, C2, "Continuous-time: not detectable", "continuous");

A3 = diag([0.3, 1.2, -0.7]);
C3 = [0 1 0];
reportDetectability(A3, C3, "Discrete-time: detectable but not observable", "discrete");

% Optional Simulink demonstration:
% If Simulink is installed, the following block creates a simple model with a
% State-Space block. The detectability test itself remains in MATLAB because
% it is an algebraic property of (A,C).
if exist("new_system", "file") == 4
    mdl = "Chapter13_Lesson3_Detectability_Model";
    if ~bdIsLoaded(mdl)
        new_system(mdl);
        open_system(mdl);

        add_block("simulink/Sources/Step", mdl + "/zero input");
        add_block("simulink/Continuous/State-Space", mdl + "/plant");
        add_block("simulink/Sinks/Scope", mdl + "/output scope");

        set_param(mdl + "/plant", ...
            "A", mat2str(A1), ...
            "B", mat2str(zeros(3,1)), ...
            "C", mat2str(C1), ...
            "D", mat2str(0));

        add_line(mdl, "zero input/1", "plant/1");
        add_line(mdl, "plant/1", "output scope/1");
        save_system(mdl);
        disp("Created optional Simulink model: " + mdl);
    end
end

function O = observabilityMatrix(A, C)
    n = size(A, 1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C * Ak]; %#ok<AGROW>
        Ak = Ak * A;
    end
end

function lambdasHidden = unobservableModes(A, C)
    O = observabilityMatrix(A, C);
    N = null(O, "r");

    if isempty(N)
        lambdasHidden = [];
        return;
    end

    Ahidden = N' * A * N;
    lambdasHidden = eig(Ahidden);
end

function tf = isDetectable(A, C, systemType)
    tol = 1e-9;
    lambdasHidden = unobservableModes(A, C);

    if isempty(lambdasHidden)
        tf = true;
        return;
    end

    if systemType == "continuous"
        tf = all(real(lambdasHidden) < -tol);
    elseif systemType == "discrete"
        tf = all(abs(lambdasHidden) < 1 - tol);
    else
        error("systemType must be continuous or discrete");
    end
end

function reportDetectability(A, C, name, systemType)
    O = observabilityMatrix(A, C);
    lambdasHidden = unobservableModes(A, C);

    fprintf("\n%s\n", name);
    fprintf("%s\n", repmat("-", 1, strlength(name)));
    fprintf("rank(O) = %d of n = %d\n", rank(O), size(A,1));

    disp("unobservable modes:");
    disp(lambdasHidden.');

    fprintf("detectable = %d\n", isDetectable(A, C, systemType));
end
