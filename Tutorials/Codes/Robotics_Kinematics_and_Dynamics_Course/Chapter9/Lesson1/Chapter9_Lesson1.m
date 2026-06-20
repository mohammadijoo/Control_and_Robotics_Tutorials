function wA = transformWrench(R, p, wB)
%TRANSFORMWRENCH Transform wrench from frame B to frame A.
%   R  : 3x3 rotation matrix from B to A
%   p  : 3x1 translation (origin of B expressed in frame A)
%   wB : 6x1 wrench [fB; mB]
%   wA : 6x1 wrench [fA; mA]

    fB = wB(1:3);
    mB = wB(4:6);

    fA = R * fB;
    mA = hat(p) * fA + R * mB;

    wA = [fA; mA];
end

function P = hat(p)
%HAT Skew-symmetric matrix for cross product: P*f == cross(p, f)
    P = [    0, -p(3),  p(2);
          p(3),     0, -p(1);
         -p(2),  p(1),     0];
end
      
