function R = Rz(theta)
    c = cos(theta); s = sin(theta);
    R = [ c -s  0;
          s  c  0;
          0  0  1 ];
end

function xB = rigid_apply(R, p, xA)
    xB = R*xA + p;
end

function [R_ca, p_ca] = rigid_compose(R_cb, p_cb, R_ba, p_ba)
    R_ca = R_cb * R_ba;
    p_ca = R_cb * p_ba + p_cb;
end

% Demo
theta = pi/4;
R = Rz(theta);
p = [0.5; -0.2; 0.1];
xA = [1;0;0];
xB = rigid_apply(R, p, xA)
