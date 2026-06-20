function R_sys = kOutOfNRel(Rc, n, k)
%KOUTOFNREL Reliability of a k-out-of-n architecture
%   Rc : scalar reliability of a single channel at time t
%   n  : total number of channels
%   k  : minimum number of working channels required

    arguments
        Rc (1,1) double {mustBeGreaterThanOrEqual(Rc,0), mustBeLessThanOrEqual(Rc,1)}
        n  (1,1) double {mustBeInteger, mustBePositive}
        k  (1,1) double {mustBeInteger, mustBePositive}
    end

    if k < 1 || k > n
        error('Need 1 <= k <= n');
    end

    R_sys = 0.0;
    for i = k:n
        c = nchoosek(n,i);
        R_sys = R_sys + c * Rc.^i .* (1.0 - Rc).^(n-i);
    end
end

% Example usage:
lambdaD = 1e-6;    % dangerous failures per hour
TI      = 8760.0;  % proof test interval (hours)
Rc      = exp(-lambdaD * TI);

R_1oo1  = kOutOfNRel(Rc, 1, 1);
R_1oo2  = kOutOfNRel(Rc, 2, 1);
disp([R_1oo1, R_1oo2])
      
