function [h, hazard] = speed_separation_monitor(d, v, reaction_time, max_decel, safety_margin)
%SPEED_SEPARATION_MONITOR Compute safety function and hazard flag.
%   d: distance (m)
%   v: relative speed (m/s), v >= 0 means approaching
%   reaction_time: T_r (s)
%   max_decel: a_max (m/s^2), must be > 0
%   safety_margin: d_safe (m)
%
%   h: safety function value
%   hazard: logical flag, true if h < 0

    if max_decel <= 0
        error('max_decel must be positive');
    end

    v = max(v, 0); % ignore negative (separating) speed for stopping distance
    d_stop = v * reaction_time + 0.5 * v.^2 / max_decel;
    h = d - d_stop - safety_margin;
    hazard = (h < 0);
end
      
