function tau_f = friction_torque(qdot, params, model)
%FRICTION_TORQUE Joint friction models: "coulomb", "viscous", "stribeck".
%
% qdot  : n x 1 joint velocity vector
% params: struct with fields Fc, Fs, vs, alpha, b (each n x 1 or scalar)
% model : string, one of 'coulomb', 'viscous', 'stribeck'

if nargin < 3
    model = 'stribeck';
end

eps = 1e-3;
qdot = qdot(:); % ensure column

switch lower(model)
    case 'coulomb'
        Fc = params.Fc;
        tau_f = Fc .* tanh(qdot / eps);

    case 'viscous'
        b = params.b;
        tau_f = b .* qdot;

    case 'stribeck'
        Fc = params.Fc;
        Fs = params.Fs;
        vs = params.vs;
        alpha = params.alpha;
        b = params.b;

        sgn = tanh(qdot / eps);
        phi = exp(-(abs(qdot) ./ vs).^alpha);
        tau_f = (Fc + (Fs - Fc) .* phi) .* sgn + b .* qdot;

    otherwise
        error('Unknown model: %s', model);
end
end
      
