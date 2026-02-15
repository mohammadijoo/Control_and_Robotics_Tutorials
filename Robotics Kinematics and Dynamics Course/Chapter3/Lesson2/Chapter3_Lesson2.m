function R = axisAngleToR(axis, theta)
    % Ensure column vector and normalize
    axis = axis(:);
    axis = axis / norm(axis);

    wx = axis(1); wy = axis(2); wz = axis(3);
    K = [    0, -wz,  wy;
          wz,    0, -wx;
         -wy,  wx,   0];

    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end

function [axis, theta] = RToAxisAngle(R)
    traceR = trace(R);
    cosTheta = (traceR - 1) / 2;
    cosTheta = max(min(cosTheta, 1), -1);
    theta = acos(cosTheta);
    epsVal = 1e-8;

    if theta < epsVal
        axis = [1; 0; 0];
        theta = 0;
        return;
    end

    axis = 1 / (2 * sin(theta)) * ...
        [R(3,2) - R(2,3);
         R(1,3) - R(3,1);
         R(2,1) - R(1,2)];
    axis = axis / norm(axis);
end
      
