% Chapter16_Lesson2.m
%
% Construction of Controllable Canonical Form (CCF) from SISO transfer-function data.
% This script implements the construction from scratch and compares with MATLAB tf2ss.

clear; clc;

% Example:
% G(s) = (2 s^2 + 5 s + 3)/(s^3 + 4 s^2 + 6 s + 8)
num = [2 5 3];
den = [1 4 6 8];

[A, B, C, D] = controllableCanonicalForm(num, den);

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('C ='); disp(C);
disp('D ='); disp(D);

% Optional Control System Toolbox comparison.
% MATLAB tf2ss may use a different canonical convention, but the transfer
% function should be input-output equivalent.
try
    [Atf, Btf, Ctf, Dtf] = tf2ss(num, den);
    disp('MATLAB tf2ss realization:');
    disp('Atf ='); disp(Atf);
    disp('Btf ='); disp(Btf);
    disp('Ctf ='); disp(Ctf);
    disp('Dtf ='); disp(Dtf);
catch ME
    disp(['tf2ss comparison skipped: ', ME.message]);
end

function [A, B, C, Dfeed] = controllableCanonicalForm(num, den)
    num = trimLeadingZeros(num);
    den = trimLeadingZeros(den);

    if abs(den(1)) < 1e-12
        error('Leading denominator coefficient must be nonzero.');
    end

    leading = den(1);
    den = den / leading;
    num = num / leading;

    n = length(den) - 1;
    if n < 1
        error('Denominator degree must be at least one.');
    end
    if length(num) > n + 1
        error('Improper transfer function: numerator degree exceeds denominator degree.');
    end

    numFull = zeros(1, n + 1);
    numFull(end - length(num) + 1:end) = num;

    Dfeed = numFull(1);
    rem = numFull - Dfeed * den;

    A = zeros(n, n);
    if n > 1
        A(1:n-1, 2:n) = eye(n-1);
    end
    A(n, :) = -fliplr(den(2:end));

    B = zeros(n, 1);
    B(n) = 1;

    C = fliplr(rem(2:end));
end

function out = trimLeadingZeros(c)
    idx = 1;
    while idx < length(c) && abs(c(idx)) < 1e-12
        idx = idx + 1;
    end
    out = c(idx:end);
end
