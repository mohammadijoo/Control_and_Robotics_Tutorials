function delta = cantilever_tip_deflection(F,L,E,I)
% delta = F L^3 / (3 E I)
delta = F * L^3 / (3*E*I);
end

function p = continuum_tip_position(kappa,phi,L)
% constant curvature tip position
if abs(kappa) < 1e-9
    p = [0;0;L];
else
    p = (1/kappa) * [ (1-cos(kappa*L))*cos(phi);
                      (1-cos(kappa*L))*sin(phi);
                       sin(kappa*L) ];
end
end
      