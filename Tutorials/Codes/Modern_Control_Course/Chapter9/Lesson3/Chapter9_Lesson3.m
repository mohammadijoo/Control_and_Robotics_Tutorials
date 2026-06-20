% Chapter9_Lesson3.m
% Modes, modal decomposition, and dominant modes for x_dot = A x, y = C x.
%
% MATLAB functions used:
%   eig, expm, ss, initial
%
% Optional Simulink section:
%   creates a minimal State-Space block model if Simulink is available.

clear; clc; close all;

A = [-0.25  1.50  0.0;
     -1.50 -0.25  0.0;
      0.00  0.00 -3.0];

B = zeros(3,1);
C = [1.0 0.0 0.4];
D = 0;

x0 = [1.0; -0.2; 2.0];
t = linspace(0,20,800);

[V,Lambda] = eig(A);
lambda = diag(Lambda);
W = inv(V);
z0 = W*x0;

disp('Eigenvalues:');
disp(lambda);

% Sort modes by largest real part.
[~,idx] = sort(real(lambda),'descend');
dominant = idx(1:2);

Xfull = zeros(length(t),3);
Xmodal = zeros(length(t),3);
Xdom = zeros(length(t),3);

for k = 1:length(t)
    tk = t(k);

    Xfull(k,:) = (expm(A*tk)*x0).';

    xModal = V*diag(exp(lambda*tk))*z0;
    Xmodal(k,:) = real(xModal).';

    xDom = zeros(3,1);
    for m = 1:length(dominant)
        i = dominant(m);
        xDom = xDom + V(:,i)*exp(lambda(i)*tk)*z0(i);
    end
    Xdom(k,:) = real(xDom).';
end

yFull = (C*Xfull.').';
yModal = (C*Xmodal.').';
yDom = (C*Xdom.').';

fprintf('Max all-mode reconstruction error: %.3e\n', max(abs(yFull-yModal)));
fprintf('Dominant modes: ');
fprintf('%d ', dominant);
fprintf('\n');

figure;
plot(t,yFull,'LineWidth',1.5); hold on;
plot(t,yDom,'--','LineWidth',1.5);
grid on;
xlabel('time [s]');
ylabel('output');
title('Full response and dominant-mode approximation');
legend('full y(t)','dominant-mode approximation');

figure;
semilogy(t,abs(yFull-yDom)+1e-14,'LineWidth',1.5);
grid on;
xlabel('time [s]');
ylabel('|full - dominant|');
title('Error caused by neglected fast mode');

% Control System Toolbox representation.
sys = ss(A,B,C,D);
[yInitial,tInitial,xInitial] = initial(sys,x0,t);

figure;
plot(tInitial,yInitial,'LineWidth',1.5);
grid on;
xlabel('time [s]');
ylabel('output');
title('Initial-condition response using ss/initial');

% Optional: create a simple Simulink State-Space model.
% This section is safe to skip on systems without Simulink.
if exist('new_system','file') == 2
    model = 'Chapter9_Lesson3_Simulink';
    if bdIsLoaded(model)
        close_system(model,0);
    end

    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Constant',[model '/zero input'],...
        'Value','0','Position',[60 80 110 110]);

    add_block('simulink/Continuous/State-Space',[model '/State-Space'],...
        'A','[-0.25 1.5 0; -1.5 -0.25 0; 0 0 -3]',...
        'B','[0;0;0]',...
        'C','[1 0 0.4]',...
        'D','0',...
        'X0','[1;-0.2;2]',...
        'Position',[180 65 300 125]);

    add_block('simulink/Sinks/Scope',[model '/Scope'],...
        'Position',[390 75 430 115]);

    add_line(model,'zero input/1','State-Space/1');
    add_line(model,'State-Space/1','Scope/1');

    set_param(model,'StopTime','20');
    save_system(model);
    fprintf('Simulink model created: %s.slx\n',model);
end
