A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

sys = ss(A,B,C,D);

t = linspace(0,5,1001);
u = ones(size(t));     % step input
x0 = [0.5; 0.0];

[y,tout,x] = lsim(sys,u,t,x0);

disp("x(tf) ="); disp(x(end,:).');
disp("y(tf) ="); disp(y(end));
      
