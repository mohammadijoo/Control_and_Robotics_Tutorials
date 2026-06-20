% Chapter17_Lesson1.m
%
% Observable Canonical Form (OCF) construction for a SISO transfer function.
% This script uses a from-scratch construction and then validates it with
% Control System Toolbox functions if they are available.

clear; clc;

% G(s) = (2 s^2 + 5 s + 3)/(s^3 + 4 s^2 + 6 s + 8)
den = [1 4 6 8];
num = [2 5 3];

[Ao, Bo, Co, Do] = observableCanonicalForm(den, num);

disp('Ao ='); disp(Ao);
disp('Bo ='); disp(Bo);
disp('Co ='); disp(Co);
disp('Do ='); disp(Do);

Oo = obsv(Ao, Co);
disp('rank(Oo) ='); disp(rank(Oo));

% Validation with Control System Toolbox.
if exist('ss', 'file') == 2 && exist('tf', 'file') == 2
    sys_ocf = ss(Ao, Bo, Co, Do);
    disp('Transfer function recovered from OCF:');
    tf(sys_ocf)
end

% Simulink use:
% 1. Insert a State-Space block.
% 2. Set A = Ao, B = Bo, C = Co, D = Do in the block parameters.
% 3. Connect a Step block to the input and a Scope block to the output.
% 4. Compare with a Transfer Fcn block using numerator num and denominator den.

function [Ao, Bo, Co, Do] = observableCanonicalForm(den, num)
    if abs(den(1)) < 1e-14
        error('Denominator leading coefficient must be nonzero.');
    end

    denLeading = den(1);
    den = den / denLeading;
    num = num / denLeading;

    n = length(den) - 1;
    if length(num) > n + 1
        error('Expected a proper transfer function.');
    end

    numPad = zeros(1, n + 1);
    numPad(end - length(num) + 1:end) = num;

    Do = numPad(1);
    remainder = numPad - Do * den;

    Ao = zeros(n, n);
    for i = 2:n
        Ao(i, i - 1) = 1;
    end
    Ao(:, n) = -flipud(den(2:end).');

    Bo = flipud(remainder(2:end).');
    Co = zeros(1, n);
    Co(n) = 1;
end
