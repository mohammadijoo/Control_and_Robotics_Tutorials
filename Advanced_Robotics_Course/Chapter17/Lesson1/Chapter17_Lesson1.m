N = 10;
m = 0.05;
k_spring = 50;
c_damp = 0.1;
g = 9.81;

% Mass and stiffness for 1D vertical motion per node (y-coordinate only)
M = m * eye(N);
K = zeros(N);
for i = 1:N-1
    e = zeros(N,1);
    e(i) = 1; e(i+1) = -1;
    K = K + k_spring * (e * e');
end

% Damping proportional to mass
C = c_damp * eye(N);

dt = 5e-4;
steps = 20000;
y = zeros(N,1);
v = zeros(N,1);

% Initialization (rope horizontal, small sag)
for i = 1:N
    y(i) = 0.0;
end

for step = 1:steps
    % Gravity
    f_ext = -g * m * ones(N,1);

    % Anchor first node
    f_ext(1) = 0;
    y(1) = 0;
    v(1) = 0;

    % Drive last node (robot gripper)
    t = step * dt;
    y(N) = 0.1 * sin(2*pi*t);
    v(N) = 0;

    % Internal elastic and damping forces: M * a = f_ext - C*v - K*y
    rhs = f_ext - C * v - K * y;
    a = M \ rhs;

    % Semi-implicit Euler
    v = v + dt * a;
    y = y + dt * v;
end

% In Simulink, put M, C, K into a second-order state-space block and
% connect the last node to the robot gripper trajectory as an input.
      
