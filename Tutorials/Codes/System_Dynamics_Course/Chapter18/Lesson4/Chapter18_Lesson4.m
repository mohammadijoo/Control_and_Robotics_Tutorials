% Chapter18_Lesson4.m
% Bond graph simulation: Se - 1 - (I,R,C) for a mass-spring-damper

clear; clc; close all;

m = 1.5; b = 1.2; k = 12.0; C = 1/k;
x0 = [0.10; 0.0]; % [q0; p0]

[t, X] = ode45(@(t,x) rhs_bg(t, x, m, b, C), [0 10], x0);

q = X(:,1);
p = X(:,2);
v = p/m;
eS = arrayfun(@Se, t);

H = 0.5*(p.^2)/m + 0.5*(q.^2)/C;
Pdiss = b*(v.^2);
Ediss = cumtrapz(t, Pdiss);
dH = gradient(H, t);
residual = eS.*v - Pdiss - dH;

disp(['Max |power residual| = ', num2str(max(abs(residual)))]);

T = table(t, q, v, H, Ediss, residual);
writetable(T, 'Chapter18_Lesson4_matlab_output.csv');

figure; plot(t,q,t,v); grid on; legend('q','v'); title('Bond-graph states');
figure; plot(t,H,t,Ediss); grid on; legend('H','E_d_i_s_s'); title('Energy accounting');
figure; plot(t,residual); grid on; title('Power-balance residual');

function dx = rhs_bg(t, x, m, b, C)
    q = x(1); p = x(2);
    v = p/m;
    eR = b*v;
    eC = q/C;
    dx = [v; Se(t) - eR - eC];
end

function y = Se(t)
    y = 2*sin(2*pi*0.8*t) + double(t >= 1.0);
end
