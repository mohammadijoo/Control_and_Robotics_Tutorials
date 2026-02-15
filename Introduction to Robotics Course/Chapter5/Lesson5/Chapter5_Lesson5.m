
function [M, V] = arm_metrics(seq, params)
% seq: 'PPP','RPP','RRP','RRP_SCARA'
% params: struct with required fields

seq = upper(seq);
M = length(seq);

switch seq
    case 'PPP'
        V = params.Lx * params.Ly * params.Lz;

    case 'RPP'
        V = pi * (params.rmax^2 - params.rmin^2) * params.Lz;

    case 'RRP'
        V = 2*pi*(cos(params.phimin) - cos(params.phimax)) * ...
            (params.rmax^3 - params.rmin^3)/3;

    case 'RRP_SCARA'
        A = pi*((params.l1 + params.l2)^2 - abs(params.l1 - params.l2)^2);
        V = A * params.Lz;

    otherwise
        V = NaN;
end
end
      