# Chapter19_Lesson5.py
# Full-System Evaluation Report Lab (Autonomous Mobile Robots)
import csv, random, math, argparse
from dataclasses import dataclass

@dataclass
class Run:
    scenario:str; seed:int; success:int; time_s:float; path_length_m:float; shortest_path_m:float
    collisions:int; min_clearance_m:float; energy_j:float; ate_rmse_m:float; rpe_rmse_m:float
    cpu_mean_ms:float; cpu_p95_ms:float

FIELDS=["scenario","seed","success","time_s","path_length_m","shortest_path_m","collisions","min_clearance_m","energy_j","ate_rmse_m","rpe_rmse_m","cpu_mean_ms","cpu_p95_ms"]

def synth(label,n_s=4,n_k=12):
    random.seed(1 if label=='baseline' else 2)
    rows=[]
    for s in range(n_s):
        hard=1+0.3*s
        for k in range(n_k):
            p=0.9-0.08*s + (0.03 if label!='baseline' else 0.0)
            succ=1 if random.random()<p else 0
            sp=15+4*s+random.random()*3
            pl=sp*(1.05+0.2*hard*random.random())
            tm=(0.95 if label!='baseline' else 1.0)*(pl/(0.55+0.1*random.random()))+6*(1-succ)
            col=1 if (random.random()<(0.25 if label=='baseline' else 0.18)) else 0
            clr=max(0.03,0.55-0.16*hard+0.18*random.random())
            ate=(0.08+0.05*hard+0.05*random.random())*(0.86 if label!='baseline' else 1.0)
            rpe=(0.03+0.02*hard+0.02*random.random())*(0.86 if label!='baseline' else 1.0)
            cm=(18+6*hard+5*random.random())*(1.06 if label!='baseline' else 1.0)
            cp=cm+8+6*random.random()
            en=18*pl+100*col+20*random.random()
            rows.append(Run(f"S{s+1}",k,succ,tm,pl,sp,col,clr,en,ate,rpe,cm,cp))
    return rows

def write_csv(path, rows):
    with open(path,'w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(FIELDS)
        for r in rows:
            w.writerow([r.scenario,r.seed,r.success,r.time_s,r.path_length_m,r.shortest_path_m,r.collisions,r.min_clearance_m,r.energy_j,r.ate_rmse_m,r.rpe_rmse_m,r.cpu_mean_ms,r.cpu_p95_ms])

def read_csv(path):
    out=[]
    with open(path,newline='',encoding='utf-8') as f:
        for i,row in enumerate(csv.DictReader(f)):
            out.append(Run(row['scenario'],int(row['seed']),int(row['success']),float(row['time_s']),float(row['path_length_m']),float(row['shortest_path_m']),int(row['collisions']),float(row['min_clearance_m']),float(row['energy_j']),float(row['ate_rmse_m']),float(row['rpe_rmse_m']),float(row['cpu_mean_ms']),float(row['cpu_p95_ms'])))
    return out

def q(xs,p):
    ys=sorted(xs); k=(len(ys)-1)*p; i=math.floor(k); j=math.ceil(k)
    return ys[i] if i==j else ys[i]*(j-k)+ys[j]*(k-i)

def mean(xs): return sum(xs)/len(xs)

def agg(rows):
    spl=[(r.shortest_path_m/max(r.path_length_m,1e-9) if r.success else 0.0) for r in rows]
    return {
      'success_rate':mean([r.success for r in rows]),
      'spl':mean(spl),
      'collision_rate':mean([1 if r.collisions>0 else 0 for r in rows]),
      'time_mean_s':mean([r.time_s for r in rows]),
      'ate_rmse_mean_m':mean([r.ate_rmse_m for r in rows]),
      'rpe_rmse_mean_m':mean([r.rpe_rmse_m for r in rows]),
      'clearance_p10_m':q([r.min_clearance_m for r in rows],0.1),
      'energy_per_m_mean':mean([r.energy_j/max(r.path_length_m,1e-9) for r in rows]),
      'cpu_mean_ms':mean([r.cpu_mean_ms for r in rows]),
      'cpu_p95_ms':mean([r.cpu_p95_ms for r in rows]),
    }

def paired(base,cand,fn):
    B={(r.scenario,r.seed):r for r in base}; C={(r.scenario,r.seed):r for r in cand}
    keys=sorted(set(B)&set(C)); return [fn(C[k])-fn(B[k]) for k in keys]

def boot_ci(ds,B=2000):
    random.seed(42); n=len(ds); ms=[]
    for _ in range(B):
        ms.append(sum(ds[random.randrange(n)] for _ in range(n))/n)
    ms.sort();
    return sum(ds)/n, ms[int(0.025*(B-1))], ms[int(0.975*(B-1))]

def report(base,cand,out='Chapter19_Lesson5_Report.md'):
    A=agg(base); C=agg(cand)
    diffs={
      'success': paired(base,cand, lambda r: float(r.success)),
      '-time': paired(base,cand, lambda r: -r.time_s),
      '-ATE': paired(base,cand, lambda r: -r.ate_rmse_m),
      'SPL': paired(base,cand, lambda r: (r.shortest_path_m/max(r.path_length_m,1e-9) if r.success else 0.0))
    }
    with open(out,'w',encoding='utf-8') as f:
        f.write('# Full-System Evaluation Report\n\n')
        f.write('|Metric|Baseline|Candidate|\n|---|---:|---:|\n')
        for k in ['success_rate','spl','collision_rate','time_mean_s','ate_rmse_mean_m','rpe_rmse_mean_m','clearance_p10_m','energy_per_m_mean','cpu_mean_ms','cpu_p95_ms']:
            f.write(f'|{k}|{A[k]:.4f}|{C[k]:.4f}|\\n')
        f.write('\n|Paired metric|Mean diff|95% CI|\n|---|---:|---:|\n')
        for name,ds in diffs.items():
            m,l,h=boot_ci(ds)
            f.write(f'|{name}|{m:.4f}|[{l:.4f}, {h:.4f}]|\\n')
    return out

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--runs',nargs=2); ap.add_argument('--out',default='Chapter19_Lesson5_Report.md'); a=ap.parse_args()
    if a.runs:
        b=read_csv(a.runs[0]); c=read_csv(a.runs[1])
    else:
        b=synth('baseline'); c=synth('candidate'); write_csv('baseline_example.csv',b); write_csv('candidate_example.csv',c)
    print(report(b,c,a.out))
