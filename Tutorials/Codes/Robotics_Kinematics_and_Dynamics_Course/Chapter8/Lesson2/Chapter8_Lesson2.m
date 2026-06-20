function w = yoshikawaManipulability(J, tol)
%YOSHIKAWAMANIPULABILITY Yoshikawa velocity manipulability index.
%   w = yoshikawaManipulability(J, tol) returns the product of the
%   nonzero singular values of J. If tol is omitted, a default tolerance
%   is used.

    if nargin < 2
        tol = 1e-6;
    end

    s = svd(J);
    s = s(s > tol);

    if isempty(s)
        w = 0.0;
    else
        w = prod(s);
    end
end


function demo_planar2R()
    l1 = 1.0;
    l2 = 0.7;
    theta1 = 0.5;
    theta2 = 1.0;

    J = jacobian2R(theta1, theta2, l1, l2);
    w = yoshikawaManipulability(J);

    fprintf("J =\n");
    disp(J);
    fprintf("Yoshikawa manipulability w = %f\n", w);
end


function J = jacobian2R(theta1, theta2, l1, l2)
    s1 = sin(theta1);
    c1 = cos(theta1);
    s12 = sin(theta1 + theta2);
    c12 = cos(theta1 + theta2);

    J = [ -l1 * s1 - l2 * s12, -l2 * s12;
           l1 * c1 + l2 * c12,  l2 * c12 ];
end
      
