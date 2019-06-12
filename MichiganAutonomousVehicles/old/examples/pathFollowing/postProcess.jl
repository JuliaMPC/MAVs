using PrettyPlots, Plots, DataFrames
#gr();
#pgfplots();
pyplot();

# TODO consider moving this to PrettyPlots
default(guidefont = font(17), tickfont = font(15), legendfont = font(12), titlefont = font(20))
c=defineCase(;(:mode=>:caseStudyPath));

plotSettings(;(:simulate=>true),(:plant=>true),(:plantOnly=>false));

description = string(
"In this test: \n",c.m.name,"\n
* m.Ni=",c.m.Ni," \n
* m.Nck=",c.m.Nck,"\n
* m.tp=",c.m.tp," \n
* m.tex=",c.m.tex,"\n
* m.max_cpu_time=",c.m.max_cpu_time," \n
* m.Nck=",c.m.Nck,"\n
* m.PredictX0=",c.m.PredictX0," \n
* m.FixedTp=",c.m.FixedTp,"\n
")
results_dir=string("test1_10_obstacles")
resultsDir!(r,results_dir;description=description);
savePlantData(n,r)

if _pretty_defaults[:simulate];
  println("Plotting the Final Results!")
  mainSim(n,r,c,pa;(:mode=>:path3))
end

# TODO make this an example
if n.r.status==:Infeasible
  #Settings(;evalConstraints=true,save=true,MPC=false);
  n.s.evalConstraints=true;n.s.save=true;n.s.MPC=false;
  postProcess!(n)
  # trouble getting a feasible solution? -> look at the constraints
  print(n.r.constraint.value)
end # then consider relaxing tolerances etc. to make it work
# then looking at the output
# These are the dual infeasibilities
#=
│ Row │ step │ x0_con   │
├─────┼──────┼──────────┤
│ 1   │ 1    │ -267.777 │
│ 2   │ 2    │ 64.6356  │
│ 3   │ 3    │ 271.738  │
│ 4   │ 4    │ 2.44213  │
│ 5   │ 5    │ 3496.5   │
=#
# These are the dual infeasibilities

# we cam try to relax the constraint on the

#=
# define tolerances
XF_tol=[NaN,NaN,NaN,NaN,NaN];
X0_tol=[0.05,0.05,0.05,0.05,0.01];
defineTolerances(n;X0_tol=X0_tol,XF_tol=XF_tol);
=#

#using PrettyPlots
#s=Settings(;save=true,MPC=false,simulate=false,format=:png);
#pSimPath(n,r,s,c,r.eval_num)
