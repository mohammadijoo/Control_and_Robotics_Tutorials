function Is = spatialInertia(m, c, Ic)
%SPATIALINERTIA Construct 6x6 spatial inertia at reference frame O.
%   m  : scalar mass
%   c  : 3x1 vector from O to CoM C
%   Ic : 3x3 inertia tensor about C

    S = skew3(c);
    I3 = eye(3);

    I11 = Ic - m * (S * S);   % or Ic + m * S * S'
    I12 = m * S;
    I21 = -m * S;
    I22 = m * I3;

    Is = [I11, I12;
          I21, I22];
end

function S = skew3(v)
%SKEW3 3x3 skew-symmetric matrix S(v) such that S(v) * x = v x x.
    vx = v(1); vy = v(2); vz = v(3);
    S = [0,   -vz,  vy;
         vz,   0,  -vx;
        -vy,   vx,  0];
end

% Example script
m = 5.0;
c = [0; 0; 0.2];
Ic = diag([0.1, 0.2, 0.15]);
Is = spatialInertia(m, c, Ic)

% Spatial velocity v = [omega; vO]
v = [0; 0; 1; 0.1; 0; 0];
h = Is * v;
T = 0.5 * v' * Is * v;
      
