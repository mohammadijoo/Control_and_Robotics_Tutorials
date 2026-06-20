% Chapter17_Lesson2.m
% CCF-OCF duality in MATLAB/Simulink.
% Requires Control System Toolbox for ss, tf, ctrb, obsv.
% Optional Simulink section requires Simulink.

clear; clc;

% G(s) = (2s^2 + 5s + 3)/(s^3 + 4s^2 + 6s + 4)
den = [1 4 6 4];
num = [2 5 3];

n = length(den) - 1;
a = fliplr(den(2:end));                    % [a0 a1 ... a_{n-1}]
b = fliplr([zeros(1, n-length(num)) num]); % [b0 ... b_{n-1}]

Ac = [zeros(n-1,1) eye(n-1); -a];
Bc = [zeros(n-1,1); 1];
Cc = b;
Dc = 0;

Ao = Ac.';
Bo = Cc.';
Co = Bc.';
Do = Dc;

disp('A_c ='); disp(Ac);
disp('A_o = A_c^T ='); disp(Ao);

Qc = ctrb(Ac, Bc);
Oo = obsv(Ao, Co);

fprintf('rank ctrb(Ac,Bc) = %d\n', rank(Qc));
fprintf('rank obsv(Ao,Co) = %d\n', rank(Oo));
fprintf('norm(Oo - Qc'')   = %.3e\n', norm(Oo - Qc.', 'fro'));

sysC = ss(Ac, Bc, Cc, Dc);
sysO = ss(Ao, Bo, Co, Do);

Gc = minreal(tf(sysC));
Go = minreal(tf(sysO));

disp('Transfer function from CCF:'); Gc
disp('Transfer function from OCF:'); Go
disp('Difference tf:'); minreal(Gc - Go)

for s = [0.5 1.0 2.0 3.0]
    Hc = Cc*((s*eye(n) - Ac)\Bc) + Dc;
    Ho = Co*((s*eye(n) - Ao)\Bo) + Do;
    fprintf('s = %.1f, H_CCF = %.10f, H_OCF = %.10f, error = %.2e\n', ...
        s, Hc, Ho, abs(Hc-Ho));
end

% Optional Simulink model generation.
createSimulinkModel = false;
if createSimulinkModel
    model = 'Chapter17_Lesson2_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Step', [model '/Step'], ...
        'Position', [50 80 80 110]);
    add_block('simulink/Continuous/State-Space', [model '/CCF_StateSpace'], ...
        'A', mat2str(Ac), 'B', mat2str(Bc), 'C', mat2str(Cc), 'D', mat2str(Dc), ...
        'Position', [150 40 310 120]);
    add_block('simulink/Continuous/State-Space', [model '/OCF_StateSpace'], ...
        'A', mat2str(Ao), 'B', mat2str(Bo), 'C', mat2str(Co), 'D', mat2str(Do), ...
        'Position', [150 160 310 240]);
    add_block('simulink/Sinks/Scope', [model '/Scope'], ...
        'Position', [390 90 430 210]);

    add_line(model, 'Step/1', 'CCF_StateSpace/1');
    add_line(model, 'Step/1', 'OCF_StateSpace/1');
    add_line(model, 'CCF_StateSpace/1', 'Scope/1');
    add_line(model, 'OCF_StateSpace/1', 'Scope/2');

    save_system(model);
end
