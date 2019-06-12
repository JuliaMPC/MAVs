using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs
using PrettyPlots,Plots
pgfplots()

c=defineCase(;(:mode=>:autoBench));

n=initializeAutonomousControl(c);

t_optJ=zeros(10);
t_optJ[1]=n.r.t_solve

for i in 2:10
  optimize!(n);
  t_optJ[i]=n.r.t_solve
end
aveJ=mean(t_optJ[2:end])

# extract data from Jiechao's tests
dfs=dataSet(:jiechao_benchmark,"Jiechao_results/")
x=dfs[1][:x];y=dfs[1][:y]
t_optM=[1.5401168711,0.6988501533,0.6917544356,0.7267272741,0.7018273181,0.6902223212,0.7143454254,0.6686120402,0.6768629488,0.6893707926];
aveM=mean(t_optM[2:end])

# plot the results
results_dir=string("benchmark1_",c.m.name,"/")
resultsDir!(n;results_name=results_dir);

# optimization times
optP=scatter(1:10,t_optJ,label="julia")
scatter!(1:10,t_optM,label="MATLAB")
yaxis!("Optimization Time (s)");
xaxis!("Evaluation Number");
title!(string("MATLAB ave. = ",round(aveM,2) ,"julia ave. = ",round(aveJ,2) ))
savefig(string(n.r.results_dir,"/","optPlot",".",:svg));

# position
pos=obstaclePlot(n,c)
plot!(n.r.X[:,1],n.r.X[:,2],label="julia")
plot!(x,y,label="MATLAB")
xaxis!(n.state.description[1]);
yaxis!(n.state.description[2]);
xlims!((200-55,200+55))
ylims!((-1,109))
savefig(string(n.r.results_dir,"/","position",".",:svg));
