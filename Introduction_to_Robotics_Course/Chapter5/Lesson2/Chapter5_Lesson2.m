
function M = mobility_spatial(L, fi)
% L: number of links including base
% fi: vector of joint DOF values, length J
J = length(fi);
M = 6*(L - 1 - J) + sum(fi);
end

% Serial example
fi_serial = ones(1,6);  % six revolute joints
M_serial = mobility_spatial(7, fi_serial)

% Stewart-platform-like count example (illustrative only)
% Suppose 13 links incl. base and platform, 18 one-DOF joints total
fi_stewart = ones(1,18);
M_stewart = mobility_spatial(13, fi_stewart)
      