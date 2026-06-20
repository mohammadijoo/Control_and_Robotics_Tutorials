function [v, w] = reactiveWallAvoider(d_front, d_safe)
% Simple reactive controller:
%   if d_front < d_safe: stop and turn
%   else: drive forward
if d_front < d_safe
    v = 0.0;
    w = 1.0;
else
    v = 0.4;
    w = 0.0;
end
end
      
