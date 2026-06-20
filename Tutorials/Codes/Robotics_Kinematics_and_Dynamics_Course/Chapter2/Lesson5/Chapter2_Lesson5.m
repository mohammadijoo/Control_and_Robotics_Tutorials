function Xi_hat = hat_se3(xi)
% xi = [omega; v] in R^6
omega = xi(1:3);
v = xi(4:6);
Xi_hat = zeros(4,4);
Xi_hat(1:3,1:3) = hat_so3(omega);
Xi_hat(1:3,4)   = v;
end

function W = hat_so3(w)
W = [   0   -w(3)  w(2);
      w(3)    0   -w(1);
     -w(2)  w(1)   0  ];
end

function g = exp_se3(xi, theta)
omega = xi(1:3);
v = xi(4:6);
wnorm = norm(omega);

g = eye(4);
if wnorm < 1e-9
    % Pure translation
    g(1:3,4) = v * theta;
    return;
end

w_hat = omega / wnorm;
W = hat_so3(w_hat);
th = theta * wnorm;

I3 = eye(3);
R = I3 + sin(th)*W + (1-cos(th))*(W*W);

A = I3*th + (1-cos(th))*W + (th-sin(th))*(W*W);
p = A * v;

g(1:3,1:3) = R;
g(1:3,4)   = p;
end

% Example usage:
xi = [0; 0; 1; 0.1; 0.2; 0];
g = exp_se3(xi, 0.5);
disp(g);
      
