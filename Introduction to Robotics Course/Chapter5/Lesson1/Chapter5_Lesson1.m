
% Quick mobility estimate from joint list
dof = containers.Map({'R','P','H','C','U','S','E'}, [1 1 1 2 2 3 3]);

nLinks = 5; % incl. ground
joints = {'R','R','P','R'};

M = 6*(nLinks-1);
for k = 1:numel(joints)
    M = M - (6 - dof(joints{k}));
end

disp(['Mobility = ', num2str(M)])

% In Simscape Multibody, you would choose joint blocks
% (Revolute Joint, Prismatic Joint, etc.) and connect links.
      