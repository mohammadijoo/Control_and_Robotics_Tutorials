function R = eulerZYXToR(phi, theta, psi)
% ZYX Euler angles to rotation matrix

cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

R = [ cpsi * cth,  cpsi * sth * sphi - spsi * cphi,  cpsi * sth * cphi + spsi * sphi;
      spsi * cth,  spsi * sth * sphi + cpsi * cphi,  spsi * sth * cphi - cpsi * sphi;
      -sth,        cth * sphi,                      cth * cphi ];
end

function omega = eulerZYXToOmega(phi, theta, phi_dot, theta_dot, psi_dot)
% Angular velocity for ZYX Euler angles

cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);

T = [1,    0,      -sth;
     0,    cphi,   cth * sphi;
     0,   -sphi,   cth * cphi];

omega = T * [phi_dot; theta_dot; psi_dot];
end
      
