% Chapter13_Lesson2.m
% Observable states and observable subspace for a continuous-time LTI system.
% Uses MATLAB base functions. If Control System Toolbox is installed, obsv(A,C)
% gives the same observability signature matrix.

clear; clc;

A = [ 0  1  0;
     -2 -3  0;
      0  0 -4];
C = [1 0 0];

n = size(A,1);
O = [];
for k = 0:n-1
    O = [O; C*(A^k)]; %#ok<AGROW>
end

fprintf('Observability signature matrix O:\n');
disp(O);
fprintf('rank(O) = %d\n', rank(O));

Nu = null(O);              % unobservable subspace basis
Vo = orth(O.');            % observable subspace basis in Euclidean coordinates
Po = Vo*Vo.';              % projection onto observable subspace
Pu = Nu*Nu.';              % projection onto unobservable subspace

fprintf('Observable subspace basis columns:\n');
disp(Vo);
fprintf('Unobservable subspace basis columns:\n');
disp(Nu);

x0 = [2; -1; 5];
fprintf('x0 =\n'); disp(x0);
fprintf('observable component =\n'); disp(Po*x0);
fprintf('unobservable component =\n'); disp(Pu*x0);
fprintf('finite output derivative signature O*x0 =\n'); disp(O*x0);
fprintf('signature of unobservable component O*(Pu*x0) =\n'); disp(O*(Pu*x0));

% Optional Simulink block generation: the measured output is y = C x.
% The input channel is set to zero because this lesson focuses on initial-state
% information, not control excitation.
try
    if license('test','Simulink')
        model = 'Chapter13_Lesson2_Simulink';
        if bdIsLoaded(model), close_system(model, 0); end
        new_system(model); open_system(model);
        add_block('simulink/Sources/Constant', [model '/zero input'], 'Value', '0');
        add_block('simulink/Continuous/State-Space', [model '/LTI system']);
        set_param([model '/LTI system'], 'A', mat2str(A), ...
            'B', mat2str(zeros(3,1)), 'C', mat2str(C), 'D', mat2str(0));
        add_block('simulink/Sinks/Scope', [model '/output scope']);
        add_line(model, 'zero input/1', 'LTI system/1');
        add_line(model, 'LTI system/1', 'output scope/1');
        save_system(model);
        fprintf('Simulink model saved as %s.slx\n', model);
    end
catch ME
    fprintf('Simulink model generation skipped: %s\n', ME.message);
end
