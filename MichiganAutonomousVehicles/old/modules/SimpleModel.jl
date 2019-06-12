module SimpleModel

using NLOptControl
using VehicleModels
using DataFrames
using Parameters

include("CaseModule.jl")
using .CaseModule

export
      initializeSimpleModel,
      updateSimpleModel,
      runSimpleModel,
      compareSimpleModels,
      resetDesignVariables

type OptData
      SA
      AX
      OBJ
      XO
      t
end

"""
initializeSimpleModels(n,c,pa);
# might have to call a single function and just have different kwargs
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function initializeSimpleModel(n,c,pa;x_min::Float64=0.0)  # can intially pass different x_min
error("this needs to be updated")
  # initialize
  s1=Settings();

  # define
  @unpack_Vpara pa # other vehicle model's inital parameters
  X0 = [x0_,y0_,psi0_,u0_];
  XF = [NaN,NaN,NaN,NaN];

  XL = [x_min,y_min,psi_min,u_min];
  XU = [x_max,y_max,psi_max,u_max];
  CL = [sa_min,-2.0];
  CU = [sa_max,2.5];
  define(KinematicBicycle;numStates=4,numControls=2,X0=X0,XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)

  # build
  configure!(n1,Nck=n.Nck;(:integrationScheme => n.integrationScheme),(:finalTimeDV => true))
  defineSolver!(n1,solver=:IPOPT)
  error("update this!")
  mdl=defineSolver!(n,c);



  # setup OCP
  params1 = [VparaKB()];   # simple vehicle parameters TODO--> are they the same?
  r1=OCPdef!(mdl1,n1,s1,params1);
  @NLobjective(mdl1, Min,  n1.tf + (r1.x[end,1]-c.x_ref)^2 + (r1.x[end,2]-c.y_ref)^2);

  # obstacles
  Q = size(c.A)[1]; # number of obstacles
  @NLparameter(mdl1, a[i=1:Q] == c.A[i]);
  @NLparameter(mdl1, b[i=1:Q] == c.B[i]);
  @NLparameter(mdl1, X_0[i=1:Q] == c.X0_obs[i]);
  @NLparameter(mdl1, Y_0[i=1:Q] == c.Y0_obs[i]);
  @NLparameter(mdl1, speed_x[i=1:Q] == c.s_x[i]);
  @NLparameter(mdl1, speed_y[i=1:Q] == c.s_y[i]);

  # obstacle postion after the intial postion
  X_obs=@NLexpression(mdl1, [j=1:Q,i=1:n1.numStatePoints], X_0[j] + speed_x[j]*n1.tV[i]);
  Y_obs=@NLexpression(mdl1, [j=1:Q,i=1:n1.numStatePoints], Y_0[j] + speed_y[j]*n1.tV[i]);

  # constraint on position
  obs_con=@NLconstraint(mdl1, [j=1:Q,i=1:n1.numStatePoints-1], 1 <= ((r1.x[(i+1),1]-X_obs[j,i])^2)/((a[j]+c.sm)^2) + ((r1.x[(i+1),2]-Y_obs[j,i])^2)/((b[j]+c.sm)^2));
  newConstraint!(r1,obs_con,:obs_con);

  # solve
  optimize!(mdl1,n1,r1,s1)

  return mdl1, n1, r1
end

# TODO  http://docs.julialang.org/en/stable/manual/parallel-computing/#clustermanagers
#=
immutable LocalManager <: ClusterManager
    np::Integer
end

function launch(manager::LocalManager, params::Dict, launched::Array, c::Condition)
    ...
end

function manage(manager::LocalManager, id::Integer, config::WorkerConfig, op::Symbol)
    ...
end
=#
"""
# create several SimpleModels to be run in parallel
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function createParallelModels(n,c,pa)
#  http://docs.julialang.org/en/stable/manual/parallel-computing/#clustermanagers
  #addprocs(3);
  Xmin=[100.,180.,100.];
  Xmax=[300.,300.,220.];
  #sm1=remotecall(initializeSimpleModel,2,(n,c,pa;x_min=Xmin[1]));
  #sm2=remotecall(initializeSimpleModel,3,(n,c,pa;x_min=Xmin[2]));
  #sm3=remotecall(initializeSimpleModel,4,(n,c,pa;x_min=Xmin[3]));
  sm1=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[1],x_max=Xmax[1])
  sm2=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[2],x_max=Xmax[2])
  sm3=@spawn initializeSimpleModel(n,c,pa;x_min=Xmin[3],x_max=Xmax[3])

  sm=[sm1,sm2,sm3];
  return sm
end



"""
# based off of predicted vehicle the states of a SimpleModel
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function updateSimpleModel(r,sm)

end


"""
# based off of predicted vehicle the states of a SimpleModel
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function runSimpleModel(r)

end


"""
# based off of predicted vehicle the states of a SimpleModel
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function compareSimpleModels(r)

end



"""
# use the best output from the SimpleModels update certain DesignVariables
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""

function resetDesignVariables()

end


end # module
