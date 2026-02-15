states = -10:10;
actions = [-1 0 1];
gamma = 0.95;

V = zeros(size(states));
for iter = 1:200
    Vnew = V;
    for s = 1:length(states)
        pos = states(s);
        Qvals = zeros(size(actions));
        for a = 1:length(actions)
            next_pos = min(10, max(-10, pos + actions(a)));
            r = (next_pos==0)*0 + (next_pos~=0)*(-1);
            s2 = find(states==next_pos);
            Qvals(a) = r + gamma*V(s2);
        end
        Vnew(s) = max(Qvals);
    end
    V = Vnew;
end

policy = zeros(size(states));
for s = 1:length(states)
   pos = states(s);
   Qvals = zeros(size(actions));
   for a = 1:length(actions)
      next_pos = min(10, max(-10, pos + actions(a)));
      r = (next_pos==0)*0 + (next_pos~=0)*(-1);
      s2 = find(states==next_pos);
      Qvals(a) = r + gamma*V(s2);
   end
   [~, idx] = max(Qvals);
   policy(s) = actions(idx);
end

disp([states' policy'])
      