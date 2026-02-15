% Simple discrete simulation loop consistent with engine stepping
dt = 1/240;
T  = 4;                 % seconds
N  = round(T/dt);

% Example: 1D point mass with gravity (as a toy proxy)
m = 1; g = 9.81;
x = zeros(N,1); v = zeros(N,1);
x(1) = 0.2; v(1) = 0;

for k = 1:N-1
    a = -g;                 % no control
    v(k+1) = v(k) + dt*a;   % semi-implicit Euler
    x(k+1) = x(k) + dt*v(k+1);
end

plot((0:N-1)*dt, x), xlabel('t'), ylabel('height')
