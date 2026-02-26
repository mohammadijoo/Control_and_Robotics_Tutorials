
% Scalar sense-think-act loop in MATLAB
a = 1.02; b = 0.05; K = 1.5;
N = 2000;
sigmaW = 0.02; sigmaV = 0.05;

x = 0;
xs = zeros(N,1); ys = zeros(N,1); us = zeros(N,1);

for k = 1:N
    r = 1.0;
    v = sigmaV * randn;
    y = x + v;          % Sense
    u = -K*y + r;       % Think
    w = sigmaW * randn;
    x = a*x + b*u + w;  % Act/Plant
    xs(k)=x; ys(k)=y; us(k)=u;
end

disp(xs(end))
      