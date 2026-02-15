function tau = saturateTorque(tau_cmd, tau_min, tau_max)
%SATURATETORQUE Componentwise saturation of joint torques.
%
% tau_cmd, tau_min, tau_max: column vectors (n x 1)

tau = tau_cmd;
for i = 1:length(tau_cmd)
    if tau(i) > tau_max(i)
        tau(i) = tau_max(i);
    elseif tau(i) < tau_min(i)
        tau(i) = tau_min(i);
    end
end
end

% Example usage in a time-stepping loop:
%
% for k = 1:N
%   tau_cmd = M(q) * qdd_des(:,k) + h(q, qd);
%   tau_act = saturateTorque(tau_cmd, tau_min, tau_max);
%   qdd = M(q) \ (tau_act - h(q, qd));
%   % Integrate qd, q ...
% end
      
