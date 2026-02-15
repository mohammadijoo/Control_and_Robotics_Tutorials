function prefix = automata_based_plan(ts, ba)
% ts: struct with fields:
%   states: N-by-2 matrix of grid coordinates
%   initial: 1-by-2 initial coordinate
%   actions: cell array of action names
%   trans: containers.Map with key '"i,j:act"' and value: M-by-2 successors
%   label: containers.Map from '"i,j"' to cell array of propositions
%
% ba: struct implementing F(goal) Buchi with:
%   initial: 'q0'
%   accepting: {'q1'}

start = {ts.initial, ba.initial}; % cell: { [i j], 'q' }

visited = containers.Map();
queue = {};
pred = containers.Map();

key = key_prod(start{1}, start{2});
visited(key) = true;
queue{end+1} = start;

acc_key = '';
while ~isempty(queue)
    cur = queue{1};
    queue(1) = [];
    s = cur{1}; q = cur{2};
    if any(strcmp(ba.accepting, q))
        acc_key = key_prod(s, q);
        break;
    end
    lbl_key = sprintf('%d,%d', s(1), s(2));
    if isKey(ts.label, lbl_key)
        sigma = ts.label(lbl_key);
    else
        sigma = {};
    end
    for a_idx = 1:numel(ts.actions)
        act = ts.actions{a_idx};
        t_key = sprintf('%d,%d:%s', s(1), s(2), act);
        if ~isKey(ts.trans, t_key), continue; end
        succs = ts.trans(t_key); % K-by-2
        for k = 1:size(succs,1)
            s_next = succs(k,:);
            q_next = buchi_delta(ba, q, sigma);
            nxt_key = key_prod(s_next, q_next);
            if ~isKey(visited, nxt_key)
                visited(nxt_key) = true;
                queue{end+1} = {s_next, q_next}; %#ok<AGROW>
                pred(nxt_key) = struct('s', s, 'q', q);
            end
        end
    end
end

if isempty(acc_key)
    prefix = {};
    return;
end

% reconstruct prefix from acc_key to start
path = {};
cur_key = acc_key;
while true
    entry = split(cur_key, ';');
    s = sscanf(entry{1}, '%d,%d');
    q = entry{2};
    path{end+1} = {s.', q}; %#ok<AGROW>
    if strcmp(cur_key, key_prod(ts.initial, ba.initial))
        break;
    end
    prev = pred(cur_key);
    cur_key = key_prod(prev.s, prev.q);
end
path = fliplr(path);
prefix = path;
end

function q_next = buchi_delta(ba, q, sigma)
has_goal = any(strcmp(sigma, 'goal'));
if strcmp(q, 'q0')
    if has_goal, q_next = 'q1'; else, q_next = 'q0'; end
else
    q_next = 'q1';
end
end

function k = key_prod(s, q)
k = sprintf('%d,%d;%s', s(1), s(2), q);
end
      
