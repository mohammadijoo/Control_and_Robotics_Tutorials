% Time-varying linear system: xdot = A(t)x + b(t)
A = @(t) [0, 1;
          -2 - 0.5*sin(t), -0.4];
b = @(t) [0; cos(2*t)];
f = @(t,x) A(t)*x + b(t);

t0 = 0; T = 8;
x0 = [1; 0];

opts = odeset('RelTol',1e-9,'AbsTol',1e-12);
[t,x] = ode45(f, [t0 T], x0, opts);

disp(['x(T) = [', num2str(x(end,1)), ', ', num2str(x(end,2)), ']']);

% LTI special case check (when A is constant):
A0 = [0 1; -2 -0.4];
x_lti = expm(A0*(T-t0))*x0;  % homogeneous solution for b(t)=0
      
