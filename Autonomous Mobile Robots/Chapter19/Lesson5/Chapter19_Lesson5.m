% Chapter19_Lesson5.m
% Full-System Evaluation Report Lab (Autonomous Mobile Robots)
function Chapter19_Lesson5()
base = readtable('baseline_example.csv','TextType','string');
cand = readtable('candidate_example.csv','TextType','string');
A = agg(base); B = agg(cand);
disp(struct2table(A)); disp(struct2table(B));
end

function S = agg(T)
spl = zeros(height(T),1);
for i=1:height(T)
    if T.success(i)==1
        spl(i)=T.shortest_path_m(i)/max(T.path_length_m(i),1e-9);
    end
end
S.success_rate=mean(T.success);
S.spl=mean(spl);
S.collision_rate=mean(T.collisions>0);
S.time_mean_s=mean(T.time_s);
S.ate_rmse_mean_m=mean(T.ate_rmse_m);
S.rpe_rmse_mean_m=mean(T.rpe_rmse_m);
S.clearance_p10_m=prctile(T.min_clearance_m,10);
S.energy_per_m_mean=mean(T.energy_j./max(T.path_length_m,1e-9));
S.cpu_mean_ms=mean(T.cpu_mean_ms);
S.cpu_p95_ms=mean(T.cpu_p95_ms);
end
