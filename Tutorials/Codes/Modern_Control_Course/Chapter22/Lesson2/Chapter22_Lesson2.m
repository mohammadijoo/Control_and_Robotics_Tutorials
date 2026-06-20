% Chapter22_Lesson2.m
% Closed-Loop State Matrix and Mode Relocation
%
% This MATLAB script demonstrates:
%   1. Acl = A - B*K.
%   2. Open-loop and closed-loop modes.
%   3. Mode relocation using place when available.
%   4. From-scratch 2x2 coefficient matching.
%   5. A short Simulink construction note.
%
% Related MATLAB tools:
%   eig, ss, initial, place, acker, Control System Toolbox, Simulink State-Space block.

clear; clc; close all;

A = [0 1;
    -2 -0.4];

B = [0;
     1];

C = [1 0];
D = 0;

disp('Open-loop modes:');
disp(eig(A));

desiredPoles = [-2 -3];

% Preferred library method: place(A,B,p).
if exist('place', 'file') == 2
    K_place = place(A, B, desiredPoles);
else
    K_place = secondOrderFeedbackByMatching(A, B, desiredPoles);
end

Acl = A - B*K_place;

disp('K:');
disp(K_place);

disp('Acl = A - B*K:');
disp(Acl);

disp('Closed-loop modes:');
disp(eig(Acl));

disp('Closed-loop characteristic polynomial coefficients:');
disp(poly(Acl));

% State-space object and initial response, if Control System Toolbox exists.
if exist('ss', 'file') == 2
    sys_cl = ss(Acl, B, C, D);
    t = linspace(0, 5, 250);
    x0 = [1; 0];
    initial(sys_cl, x0, t);
    title('Closed-loop initial response: x_dot = (A - B K)x');
end

% Simulink implementation note:
%   1. Put a State-Space block with A = A, B = B, C = eye(2), D = zeros(2,1).
%   2. Feed its state output x into a Gain block K.
%   3. Use a Sum block to implement u = -K*x + r.
%   4. Feed u back into the State-Space block input.
% Programmatic construction depends on local Simulink availability and model
% preferences, so this script keeps the model instructions explicit.

function K = secondOrderFeedbackByMatching(A, B, desiredPoles)
    % From-scratch 2x2 SISO coefficient matching.
    p1 = desiredPoles(1);
    p2 = desiredPoles(2);

    desiredA1 = -(p1 + p2);
    desiredA0 = p1 * p2;

    coeffsFor = @(k1, k2) charCoeffs2(A - B*[k1 k2]);

    c00 = coeffsFor(0, 0);
    c10 = coeffsFor(1, 0);
    c01 = coeffsFor(0, 1);

    M = [(c10 - c00), (c01 - c00)];
    target = [desiredA1; desiredA0];
    k = M \ (target - c00);
    K = k.';
end

function c = charCoeffs2(M)
    % det(sI - M) = s^2 - trace(M)s + det(M).
    c = [-trace(M); det(M)];
end
