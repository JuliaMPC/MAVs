using PrettyPlots, Plots, DataFrames
gr();
#pgfplots();
#pyplot();

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
results_dir=string("testE_",c.m.name,"_",c.m.solver,"/")
resultsDir!(r,results_dir;description=description);


println("Plotting the Final Results!")
s=Settings(;reset=false,save=true,simulate=true,MPC=false,format=:png);
#pmain(n,r,s,c)
#pp(n,r,s,c)
mainSimPath(n,r,s,c,pa)
if r.dfs_opt[r.eval_num-1][:status]==:Infeasible
  s=Settings(;evalConstraints=true,save=true,MPC=false,simulate=false,format=:png);
  postProcess(n,r,s)
  # trouble getting a feasible solution? -> look at the constraints
  print(r.constraint.value)
end # then consider relaxing tolerances etc. to make it work
