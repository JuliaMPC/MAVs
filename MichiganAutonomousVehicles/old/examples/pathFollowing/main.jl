using NLOptControl
using VehicleModels
using Parameters
using DataFrames
using MAVs

c=defineCase(;(:mode=>:caseStudyPath));
setMisc!(c;mpc_max_iter=400);
setWeights!(c;sr=0.05,path=10.0);
#=
c.o.X0=c.o.X0[1:8];
c.o.Y0=c.o.Y0[1:8];
c.o.A=c.o.A[1:8];
c.o.B=c.o.B[1:8];
c.o.status=c.o.status[1:8];
c.o.s_x=c.o.s_x[1:8];
c.o.s_y=c.o.s_y[1:8];
=#
mdl,n,r,params=initializePathFollowing(c);
global pa=params[1];
global s=Settings(;reset=false,save=true,MPC=true);

driveStraight!(n,pa,r,s)
for ii=2:n.mpc.max_iter
    if ((r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < n.mpc.tex*r.dfs_plant[1][:ux][1] # || sum(getvalue(dt)) < 0.0001
      println("Track Complete! \n"); n.mpc.goal_reached=true; break;
    end
    println("Running model for the: ",r.eval_num," time");
    updatePathParams!(n,r,c,params);             # update model parameters
    status=autonomousControl!(mdl,n,r,s,pa);     # rerun optimization
    if status==:Optimal || status==:Suboptimal || status==:UserLimit
      println("Passing Optimized Signals to 3DOF Vehicle Model");
      n.mpc.t0_actual=(r.eval_num-1)*n.mpc.tex;  # external so that it can be updated easily in PathFollowing
      simPlant!(n,r,s,pa,n.X0,r.t_ctr+n.mpc.t0,r.U,n.mpc.t0_actual,r.eval_num*n.mpc.tex)
    elseif status==:Infeasible
      println("\n FINISH:Passing PREVIOUS Optimized Signals to 3DOF Vehicle Model \n"); break;
    else
      println("\n There status is nor Optimal or Infeaible -> create logic for this case!\n"); break;
    end
  if r.eval_num==n.mpc.max_iter
    warn(" \n The maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration) \n")
  end
end
include("postProcess.jl");
